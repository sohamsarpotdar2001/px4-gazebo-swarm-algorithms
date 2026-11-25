#include <drone_swarm/algo_base.hpp>

struct DroneBee : public BaseDroneAgent {
    Vec3 goal_slot_offset;
    Vec3 food_source{0.0, 0.0, -5.0};
    double obj_val = std::numeric_limits<double>::infinity();
    double fitness = 0.0;
    int trials = 0;
};

class Px4AbcController : public SwarmBaseController<DroneBee> {
public:
    explicit Px4AbcController(int cli_drone_count = -1) : SwarmBaseController<DroneBee>("px4_abc_controller", "abc", cli_drone_count) {
        goal_spread_radius_ = this->declare_parameter<double>("goal_spread_radius", 1.6);
        abandon_limit_ = this->declare_parameter<int>("abandon_limit", 12);
        k_track_ = this->declare_parameter<double>("k_track", 0.95);
        k_repel_ = this->declare_parameter<double>("k_repel", 3.50);

        for (int i = 0; i < num_drones_; ++i) {
            if (num_drones_ > 1) {
                double angle = (2.0 * M_PI * i) / static_cast<double>(num_drones_);
                drones_[i].goal_slot_offset = {
                    goal_spread_radius_ * std::cos(angle),
                    goal_spread_radius_ * std::sin(angle),
                    0.0
                };
            }
        }
    }

protected:
    void goal_subscriber_cb(const geometry_msgs::msg::Point::SharedPtr msg) override {
        const double half_box = box_size_ * 0.5;
        const double margin = goal_spread_radius_ + 0.4;

        active_goal_.x = std::clamp(msg->x, -half_box + margin, half_box - margin);
        active_goal_.y = std::clamp(msg->y, -half_box + margin, half_box - margin);
        active_goal_.z = std::clamp(msg->z, -6.0 - half_box + margin, -6.0 + half_box - margin);

        if (!has_external_goal_) {
            RCLCPP_INFO(this->get_logger(), "First /goal received! Enabling offboard control sequence.");
        }

        has_external_goal_ = true;
        goal_reached_reported_ = false;
        reset_food_sources();

        RCLCPP_INFO(this->get_logger(),
                    "Received new /goal waypoint (clamped to 10m box NED): (%.2f, %.2f, %.2f)",
                    active_goal_.x, active_goal_.y, active_goal_.z);
    }

    void reset() override {
        reset_food_sources();
    }

    double evaluate_objective(const DroneBee& d, const Vec3& candidate) const {
        Vec3 target_i = active_goal_ + d.goal_slot_offset;
        double dist_sq = (candidate - target_i).length_sq();

        double conflict_penalty = 0.0;
        for (const auto& other : drones_) {
            if (other.instance_id == d.instance_id) continue;
            double sep = (candidate - other.food_source).length();
            if (sep < safe_separation_) {
                double pen = safe_separation_ - sep;
                conflict_penalty += 25.0 * pen * pen;
            }
        }
        return dist_sq + conflict_penalty;
    }

    static double compute_fitness(double obj) {
        return (obj >= 0.0) ? (1.0 / (1.0 + obj)) : (1.0 + std::abs(obj));
    }

    Vec3 clamp_to_box(const Vec3& p) const {
        const double half_box = box_size_ * 0.5;
        const double margin   = 0.5;
        return {
            std::clamp(p.x, -half_box + margin, half_box - margin),
            std::clamp(p.y, -half_box + margin, half_box - margin),
            std::clamp(p.z, -6.0 - half_box + margin, -6.0 + half_box - margin)
        };
    }

    void reset_food_sources() {
        for (auto& d : drones_) {
            d.food_source = clamp_to_box(d.global_pos);
            d.obj_val     = evaluate_objective(d, d.food_source);
            d.fitness     = compute_fitness(d.obj_val);
            d.trials      = 0;
        }
    }

    void explore_neighbor_food_source(size_t i) {
        if (drones_.empty()) return;
        const size_t sn = drones_.size();

        std::uniform_int_distribution<int> dim_dist(0, 2);
        std::uniform_real_distribution<double> phi_dist(-1.0, 1.0);

        int j = dim_dist(rng_);
        double phi = phi_dist(rng_);

        Vec3 candidate = drones_[i].food_source;
        if (sn > 1) {
            std::uniform_int_distribution<size_t> partner_dist(0, sn - 2);
            size_t k = partner_dist(rng_);
            if (k >= i) ++k;

            double slot_diff = (drones_[i].food_source[j] - drones_[i].goal_slot_offset[j]) -
                               (drones_[k].food_source[j] - drones_[k].goal_slot_offset[j]);
            candidate[j] = drones_[i].food_source[j] + phi * slot_diff;
        } else {
            Vec3 target_0 = active_goal_ + drones_[0].goal_slot_offset;
            candidate[j] = drones_[i].food_source[j] + std::abs(phi) * (target_0[j] - drones_[i].food_source[j]);
        }

        Vec3 target_i = active_goal_ + drones_[i].goal_slot_offset;
        candidate[j] += 0.35 * (target_i[j] - candidate[j]);
        candidate = clamp_to_box(candidate);

        double cand_obj = evaluate_objective(drones_[i], candidate);
        double cand_fit = compute_fitness(cand_obj);

        if (cand_fit > drones_[i].fitness) {
            drones_[i].food_source = candidate;
            drones_[i].obj_val     = cand_obj;
            drones_[i].fitness     = cand_fit;
            drones_[i].trials      = 0;
        } else {
            ++drones_[i].trials;
        }
    }

    void run_abc_cycle() {
        const size_t sn = drones_.size();
        if (sn == 0) return;

        for (size_t i = 0; i < sn; ++i) {
            drones_[i].obj_val = evaluate_objective(drones_[i], drones_[i].food_source);
            drones_[i].fitness = compute_fitness(drones_[i].obj_val);
        }

        for (size_t i = 0; i < sn; ++i) {
            explore_neighbor_food_source(i);
        }

        double total_fitness = 0.0;
        for (const auto& d : drones_) total_fitness += d.fitness;

        std::uniform_real_distribution<double> uni01(0.0, 1.0);
        size_t onlookers_dispatched = 0;
        size_t idx = 0;
        size_t safety_guard = 0;
        while (onlookers_dispatched < sn && safety_guard < sn * 10) {
            double prob = (total_fitness > 1e-12) ? (drones_[idx].fitness / total_fitness) : (1.0 / sn);
            if (uni01(rng_) < prob) {
                explore_neighbor_food_source(idx);
                ++onlookers_dispatched;
            }
            idx = (idx + 1) % sn;
            ++safety_guard;
        }

        std::uniform_real_distribution<double> jitter_dist(-1.2, 1.2);
        for (size_t i = 0; i < sn; ++i) {
            if (drones_[i].trials >= abandon_limit_) {
                Vec3 target_i = active_goal_ + drones_[i].goal_slot_offset;
                Vec3 p{target_i.x + jitter_dist(rng_), target_i.y + jitter_dist(rng_), target_i.z + 0.5 * jitter_dist(rng_)};
                Vec3 scout_pos = clamp_to_box(p);
                drones_[i].food_source = scout_pos;
                drones_[i].obj_val     = evaluate_objective(drones_[i], scout_pos);
                drones_[i].fitness     = compute_fitness(drones_[i].obj_val);
                drones_[i].trials      = 0;
            }
        }
    }

    void compute_velocities() override {
        if (tick_count_ % 10 == 0) {
            run_abc_cycle();
        }

        for (int i = 0; i < num_drones_; ++i) {
            const Vec3& p_i = drones_[i].global_pos;

            Vec3 to_food = drones_[i].food_source - p_i;
            Vec3 v_abc   = (to_food * k_track_).limited(max_speed_ * 0.75);

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
                v_abc + u_repel + bound_force
            );
        }
        check_and_report_goal_reached(compute_centroid());
    }

    std::random_device rd;
    std::mt19937 rng_{rd()};

private:
    double goal_spread_radius_, abandon_limit_, k_track_, k_repel_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4AbcController>(argc, argv);
}
