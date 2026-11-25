#include <drone_swarm/algo_base.hpp>

class Px4BoidsController : public SwarmBaseController<BaseDroneAgent> {
public:
    explicit Px4BoidsController(int cli_drone_count = -1)
        : SwarmBaseController<BaseDroneAgent>("px4_boids_controller", "boids", cli_drone_count) {
        perception_radius_ = this->declare_parameter<double>("perception_radius", 25.0);
        max_force_ = this->declare_parameter<double>("max_force", 0.45);
        w_separation_ = this->declare_parameter<double>("w_separation", 1.0);
        w_alignment_ = this->declare_parameter<double>("w_alignment", 0.5);
        w_cohesion_ = this->declare_parameter<double>("w_cohesion", 0.5);
        w_goal_ = this->declare_parameter<double>("w_goal", 0.85);
    }

protected:
    void compute_velocities() override {
        const double perc_sq = perception_radius_ * perception_radius_;

        for (int i = 0; i < num_drones_; ++i) {
            const Vec3& p_i = drones_[i].global_pos;
            Vec3 avg_vel{0.0, 0.0, 0.0};
            Vec3 center_of_mass{0.0, 0.0, 0.0};
            Vec3 sep_vel{0.0, 0.0, 0.0};
            int neighbors = 0;

            double min_neighbor_dist = std::numeric_limits<double>::infinity();
            for (int j = 0; j < num_drones_; ++j) {
                if (i == j || !drones_[j].has_odom) continue;

                Vec3 diff = drones_[j].global_pos - p_i;
                min_neighbor_dist = std::min(diff.length(), min_neighbor_dist);
                if (diff.length_sq() < perc_sq) {
                    avg_vel += drones_[j].vel;
                    center_of_mass += drones_[j].global_pos;
                    ++neighbors;
                }
                if (min_neighbor_dist < safe_separation_ && min_neighbor_dist > 0.0) {
                    sep_vel -= (diff/min_neighbor_dist) * (w_separation_ * (safe_separation_ - min_neighbor_dist) / min_neighbor_dist);
                }
            }

            Vec3 align_force{0.0, 0.0, 0.0};
            Vec3 coh_force{0.0, 0.0, 0.0};
            if (neighbors > 0) {
                double inv_n = 1.0 / static_cast<double>(neighbors);
                align_force = (avg_vel * inv_n) - drones_[i].vel;
                coh_force = (center_of_mass * inv_n) - p_i;
            }

            double crowd_scale = (min_neighbor_dist < safe_separation_) ? 0.15 : 1.0;
            Vec3 goal_force = (active_goal_ - p_i) * (w_goal_ * crowd_scale);
            coh_force = coh_force * crowd_scale;
            align_force = align_force * crowd_scale;

            Vec3 boids_accel = align_force.limited(max_force_) * w_alignment_ + coh_force.limited(max_force_) * w_cohesion_;

            Vec3 v_boids = (drones_[i].vel * 0.65 + boids_accel + goal_force).limited(max_speed_);
            Vec3 bound_force = compute_wall_force(drones_[i].global_pos);
            Vec3 v_next = (v_boids + sep_vel + bound_force);

            drones_[i].cmd_vel = apply_safety_limits(drones_[i].global_pos, v_next);
        }

        check_and_report_goal_reached(compute_centroid());
    }

private:
    double perception_radius_, max_force_, w_separation_, w_alignment_, w_cohesion_, w_goal_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4BoidsController>(argc, argv);
}
