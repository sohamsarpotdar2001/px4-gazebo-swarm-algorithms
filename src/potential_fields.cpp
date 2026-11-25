#include <drone_swarm/algo_base.hpp>

class Px4PotentialFieldController : public SwarmBaseController<BaseDroneAgent> {
public:
    explicit Px4PotentialFieldController(int cli_drone_count = -1) : SwarmBaseController<BaseDroneAgent>("px4_potential_fields_controller", "potential_fields", cli_drone_count) {
        k_att_          = this->declare_parameter<double>("k_att", 0.7);
        d_att_thresh_   = this->declare_parameter<double>("d_att_thresh", 1.5);
        k_rep_agent_    = this->declare_parameter<double>("k_rep_agent", 2.2);
        k_rep_wall_     = this->declare_parameter<double>("k_rep_wall", 1.4);
        wall_margin_    = this->declare_parameter<double>("wall_margin", 1.4);
        k_damp_         = this->declare_parameter<double>("k_damp", 0.25);
    }

protected:
    double wall_repulsive_1d(double dist) const {
        if (dist >= wall_margin_) return 0.0;
        double d_safe = std::max(dist, 0.15);
        return k_rep_wall_ * (1.0 / d_safe - 1.0 / wall_margin_) / (d_safe * d_safe);
    }

    void compute_velocities() override {
        const double half_box = box_size_ * 0.5;
        const double x_min = -half_box, x_max = half_box;
        const double y_min = -half_box, y_max = half_box;
        const double z_min = -6.0 - half_box, z_max = -6.0 + half_box;

        for (int i = 0; i < num_drones_; ++i) {
            Vec3 to_goal = active_goal_ - drones_[i].global_pos;
            double d_goal = to_goal.length();
            Vec3 f_att{0.0, 0.0, 0.0};
            if (d_goal > 1e-6) {
                if (d_goal <= d_att_thresh_) {
                    f_att = to_goal * k_att_;
                } else {
                    f_att = (to_goal / d_goal) * (k_att_ * d_att_thresh_);
                }
            }

            Vec3 f_rep_agents{0.0, 0.0, 0.0};
            for (int j = 0; j < num_drones_; ++j) {
                if (i == j || !drones_[j].has_odom) continue;

                Vec3 diff = drones_[i].global_pos - drones_[j].global_pos;
                double d_ij = diff.length();

                if (d_ij < safe_separation_) {
                    double d_safe = std::max(d_ij, 0.25);
                    Vec3 dir = (d_ij > 1e-6) ? (diff / d_ij) : Vec3{(i < j ? 1.0 : -1.0), 0.0, 0.0};
                    double mag = k_rep_agent_ * (1.0 / d_safe - 1.0 / safe_separation_) / (d_safe * d_safe);
                    f_rep_agents += dir * mag;
                }
            }

            Vec3 f_rep_walls{0.0, 0.0, 0.0};
            f_rep_walls.x += wall_repulsive_1d(drones_[i].global_pos.x - x_min);
            f_rep_walls.x -= wall_repulsive_1d(x_max - drones_[i].global_pos.x);
            f_rep_walls.y += wall_repulsive_1d(drones_[i].global_pos.y - y_min);
            f_rep_walls.y -= wall_repulsive_1d(y_max - drones_[i].global_pos.y);
            f_rep_walls.z += wall_repulsive_1d(drones_[i].global_pos.z - z_min);
            f_rep_walls.z -= wall_repulsive_1d(z_max - drones_[i].global_pos.z);

            Vec3 v_cmd = (f_att + f_rep_agents + f_rep_walls - drones_[i].vel * k_damp_);

            drones_[i].cmd_vel = apply_safety_limits(drones_[i].global_pos, v_cmd);
        }

        check_and_report_goal_reached(compute_centroid());
    }

private:
    double k_att_, d_att_thresh_, k_rep_agent_, k_rep_wall_, wall_margin_, k_damp_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4PotentialFieldController>(argc, argv);
}
