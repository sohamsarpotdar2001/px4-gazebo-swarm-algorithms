#include <drone_swarm/algo_base.hpp>

struct DroneAnt : public BaseDroneAgent {
    Vec3 next_waypoint{0.0, 0.0, -5.0};
    std::vector<int> visited_nodes;
    double path_length = 0.0;
};

class Px4AcoController : public SwarmBaseController<DroneAnt> {
public:
    explicit Px4AcoController(int cli_drone_count = -1) : SwarmBaseController<DroneAnt>("px4_aco_controller", "aco", cli_drone_count) {
        alpha_pheromone_ = this->declare_parameter<double>("alpha_pheromone", 1.0);
        beta_heuristic_ = this->declare_parameter<double>("beta_heuristic", 2.5);
        evaporation_rho_ = this->declare_parameter<double>("evaporation_rho", 0.04);
        pheromone_q_ = this->declare_parameter<double>("pheromone_q", 5.0);
        k_repel_ = this->declare_parameter<double>("k_repel", 3.50);

        pheromone_.assign(kGridTotal, 1.0);
        seed_goal_pheromone();

        for (auto& d : drones_) {
            d.next_waypoint = active_goal_;
        }
    }

protected:
    void goal_subscriber_cb(const geometry_msgs::msg::Point::SharedPtr msg) override {
        const double half_box = box_size_ * 0.5;
        const double margin   = 0.5;

        active_goal_.x = std::clamp(msg->x, -half_box + margin, half_box - margin);
        active_goal_.y = std::clamp(msg->y, -half_box + margin, half_box - margin);
        active_goal_.z = std::clamp(msg->z, -6.0 - half_box + margin, -6.0 + half_box - margin);

        std::fill(pheromone_.begin(), pheromone_.end(), 1.0);
        seed_goal_pheromone();
        for (auto& d : drones_) {
            d.visited_nodes.clear();
            d.path_length = 0.0;
            d.next_waypoint = select_next_aco_waypoint(d);
        }

        if (!has_external_goal_) {
            RCLCPP_INFO(this->get_logger(), "First /goal received! Enabling offboard control sequence.");
        }

        has_external_goal_ = true;
        goal_reached_reported_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "Received new /goal waypoint (clamped to %.1fm box NED): (%.2f, %.2f, %.2f)",
                    box_size_, active_goal_.x, active_goal_.y, active_goal_.z);
    }

    static constexpr int kGridN = 5;
    static constexpr int kGridTotal = kGridN * kGridN * kGridN;

    Vec3 grid_to_world(int ix, int iy, int iz) const {
        const double step = box_size_ / static_cast<double>(kGridN);
        const double half_span = 0.5 * step * (kGridN - 1);
        return {
            -half_span + ix * step,
            -half_span + iy * step,
            -6.0 - half_span + iz * step
        };
    }

    int flat_idx(int ix, int iy, int iz) const {
        return (ix * kGridN + iy) * kGridN + iz;
    }

    std::array<int, 3> world_to_grid(const Vec3& p) const {
        const double step = box_size_ / static_cast<double>(kGridN);
        const double half_span = 0.5 * step * (kGridN - 1);
        int ix = std::clamp(static_cast<int>(std::round((p.x + half_span) / step)), 0, kGridN - 1);
        int iy = std::clamp(static_cast<int>(std::round((p.y + half_span) / step)), 0, kGridN - 1);
        int iz = std::clamp(static_cast<int>(std::round((p.z + 6.0 + half_span) / step)), 0, kGridN - 1);
        return {ix, iy, iz};
    }

    void seed_goal_pheromone() {
        for (int ix = 0; ix < kGridN; ++ix) {
            for (int iy = 0; iy < kGridN; ++iy) {
                for (int iz = 0; iz < kGridN; ++iz) {
                    Vec3 node_pos = grid_to_world(ix, iy, iz);
                    double dist = (node_pos - active_goal_).length();
                    pheromone_[flat_idx(ix, iy, iz)] = 1.0 + 3.0 / (1.0 + dist);
                }
            }
        }
    }

    Vec3 select_next_aco_waypoint(DroneAnt& ant) {
        if ((ant.global_pos - active_goal_).length() < 1.8) {
            return active_goal_;
        }

        auto [cx, cy, cz] = world_to_grid(ant.global_pos);
        int cur_flat = flat_idx(cx, cy, cz);

        if (ant.visited_nodes.empty() || ant.visited_nodes.back() != cur_flat) {
            if (!ant.visited_nodes.empty()) {
                int prev = ant.visited_nodes.back();
                int px = prev / (kGridN * kGridN);
                int py = (prev / kGridN) % kGridN;
                int pz = prev % kGridN;
                ant.path_length += (grid_to_world(cx, cy, cz) - grid_to_world(px, py, pz)).length();
            }
            ant.visited_nodes.push_back(cur_flat);
        }

        struct Candidate {
            int idx;
            Vec3 pos;
            double weight;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(26);
        double total_weight = 0.0;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    int nx = cx + dx, ny = cy + dy, nz = cz + dz;
                    if (nx < 0 || nx >= kGridN || ny < 0 || ny >= kGridN || nz < 0 || nz >= kGridN) continue;

                    int n_idx = flat_idx(nx, ny, nz);
                    Vec3 n_pos = grid_to_world(nx, ny, nz);

                    double d_goal = (n_pos - active_goal_).length();
                    double eta = 1.0 / (d_goal + 0.35);

                    double crowding_penalty = 1.0;
                    for (const auto& other : drones_) {
                        if (other.instance_id == ant.instance_id || !other.has_odom) continue;
                        if ((n_pos - other.global_pos).length() < safe_separation_) {
                            crowding_penalty *= 0.25;
                        }
                    }

                    bool recently_visited = std::find(
                        ant.visited_nodes.begin(), ant.visited_nodes.end(), n_idx) != ant.visited_nodes.end();
                    double tabu_factor = recently_visited ? 0.20 : 1.0;

                    double tau = pheromone_[n_idx];
                    double w = std::pow(tau, alpha_pheromone_) * std::pow(eta, beta_heuristic_) *
                               crowding_penalty * tabu_factor;

                    candidates.push_back({n_idx, n_pos, w});
                    total_weight += w;
                }
            }
        }

        if (candidates.empty() || total_weight <= 1e-12) {
            return active_goal_;
        }

        std::uniform_real_distribution<double> dist(0.0, total_weight);
        double spin = dist(rng_);
        double accum = 0.0;
        for (const auto& c : candidates) {
            accum += c.weight;
            if (spin <= accum) {
                return c.pos;
            }
        }
        return candidates.back().pos;
    }

    void deposit_and_evaporate_pheromones() {
        // 1. Global Pheromone Evaporation
        for (double& tau : pheromone_) {
            tau = std::max(0.1, (1.0 - evaporation_rho_) * tau);
        }

        // 2. Pheromone Deposit by Ants near active_goal_ (Delta_tau = Q / L_k)
        for (auto& ant : drones_) {
            if ((ant.global_pos - active_goal_).length() < 1.2 && !ant.visited_nodes.empty()) {
                double delta_tau = pheromone_q_ / std::max(ant.path_length, 1.0);
                for (int node_idx : ant.visited_nodes) {
                    pheromone_[node_idx] = std::min(25.0, pheromone_[node_idx] + delta_tau);
                }
                ant.visited_nodes.clear();
                ant.path_length = 0.0;
            }
        }
    }

    void compute_velocities() override {
        if (tick_count_ % 10 == 0) {
            deposit_and_evaporate_pheromones();
        }

        for (int i = 0; i < num_drones_; ++i) {
            auto& ant = drones_[i];
            const Vec3& p_i = ant.global_pos;

            if ((ant.next_waypoint - p_i).length() < 0.65 || tick_count_ % 30 == 0) {
                ant.next_waypoint = select_next_aco_waypoint(ant);
            }

            Vec3 to_wp = ant.next_waypoint - p_i;
            Vec3 v_aco = (to_wp * 0.95).limited(max_speed_);

            Vec3 u_repel{0.0, 0.0, 0.0};
            for (int j = 0; j < num_drones_; ++j) {
                if (i == j || !drones_[j].has_odom) continue;

                Vec3 diff = drones_[j].global_pos - p_i;
                double d = diff.length();

                if (d < safe_separation_ && d > 1e-3) {
                    u_repel -= (diff / d) * (k_repel_ * (safe_separation_ - d) / d);
                }
            }

            Vec3 bound_force = compute_wall_force(p_i);

            drones_[i].cmd_vel = apply_safety_limits(p_i,
                v_aco + u_repel + bound_force
            );
        }

        check_and_report_goal_reached(compute_centroid());
    }

    std::vector<double> pheromone_;
    std::random_device rd;
    std::mt19937 rng_{rd()};

private:
    double alpha_pheromone_, beta_heuristic_, evaporation_rho_, pheromone_q_, k_repel_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4AcoController>(argc, argv);
}
