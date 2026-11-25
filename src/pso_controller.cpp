#include <drone_swarm/algo_base.hpp>

struct DroneParticle : public BaseDroneAgent {
    Vec3 p_best{0.0, 0.0, 0.0};
    double p_best_cost = std::numeric_limits<double>::infinity();
    double r1 = 0.5;
    double r2 = 0.5;
};

class Px4PsoController : public SwarmBaseController<DroneParticle> {
public:
    explicit Px4PsoController(int cli_drone_count = -1) : SwarmBaseController<DroneParticle>("px4_pso_controller", "pso", cli_drone_count) {
        inertia_w_     = this->declare_parameter<double>("inertia_w", 0.30);
        c1_cognitive_  = this->declare_parameter<double>("c1_cognitive", 0.35);
        c2_social_     = this->declare_parameter<double>("c2_social", 0.45);
        c_goal_guide_  = this->declare_parameter<double>("c_goal_guide", 0.70);
        k_repel_       = this->declare_parameter<double>("k_repel", 3.50);
    }

protected:
    void goal_subscriber_cb(const geometry_msgs::msg::Point::SharedPtr msg) override {
        const double half_box = box_size_ * 0.5;
        const double margin   = 0.5;

        active_goal_.x = std::clamp(msg->x, -half_box + margin, half_box - margin);
        active_goal_.y = std::clamp(msg->y, -half_box + margin, half_box - margin);
        active_goal_.z = std::clamp(msg->z, -6.0 - half_box + margin, -6.0 + half_box - margin);

        if (!has_external_goal_) {
            RCLCPP_INFO(this->get_logger(), "First /goal received! Enabling offboard control sequence.");
        }

        has_external_goal_ = true;
        goal_reached_reported_ = false;
        reset_pso_memory();

        RCLCPP_INFO(this->get_logger(),
                    "Received new /goal waypoint (clamped to %.1fm box NED): (%.2f, %.2f, %.2f)",
                    box_size_, active_goal_.x, active_goal_.y, active_goal_.z);
    }

    double evaluate_objective(const Vec3& pos) const {
        return (pos - active_goal_).length_sq();
    }

    void reset_pso_memory() {
        g_best_cost_ = std::numeric_limits<double>::infinity();
        for (auto& d : drones_) {
            d.p_best = d.global_pos;
            d.p_best_cost = evaluate_objective(d.global_pos);
            if (d.p_best_cost < g_best_cost_) {
                g_best_cost_ = d.p_best_cost;
                g_best_ = d.global_pos;
            }
        }
    }

    void compute_velocities() override {
        if (tick_count_ % 10 == 0) {
            for (auto& d : drones_) {
                d.r1 = uni_dist_(rng_);
                d.r2 = uni_dist_(rng_);
            }
        }

        for (auto& d : drones_) {
            double cost = evaluate_objective(d.global_pos);
            if (cost < d.p_best_cost) {
                d.p_best_cost = cost;
                d.p_best = d.global_pos;
            }
            if (cost < g_best_cost_) {
                g_best_cost_ = cost;
                g_best_ = d.global_pos;
            }
        }

        for (int i = 0; i < num_drones_; ++i) {
            const Vec3& p_i = drones_[i].global_pos;

            Vec3 v_inertia = drones_[i].vel * inertia_w_;
            Vec3 v_cognitive = (drones_[i].p_best - p_i) * (c1_cognitive_ * drones_[i].r1);
            Vec3 v_social = (g_best_ - p_i) * (c2_social_ * drones_[i].r2);
            Vec3 v_guide = (active_goal_ - p_i) * c_goal_guide_;

            Vec3 v_pso = (v_inertia + v_cognitive + v_social + v_guide).limited(max_speed_);

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
                v_pso + u_repel + bound_force
            );
        }

        check_and_report_goal_reached(compute_centroid());
    }

    Vec3 g_best_{0.0, 0.0, -5.0};
    double g_best_cost_ = std::numeric_limits<double>::infinity();

    std::random_device rd;
    std::mt19937 rng_{rd()};
    std::uniform_real_distribution<double> uni_dist_{0.2, 1.0};

private:
    double inertia_w_, c1_cognitive_, c2_social_, c_goal_guide_, k_repel_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4PsoController>(argc, argv);
}
