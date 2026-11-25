#include <drone_swarm/algo_base.hpp>

class Px4ConsensusGoalController : public SwarmBaseController<BaseDroneAgent> {
public:
    explicit Px4ConsensusGoalController(int cli_drone_count = -1) : SwarmBaseController<BaseDroneAgent>("px4_consensus_goal_controller", "consensus", cli_drone_count) {
        comm_radius_ = this->declare_parameter<double>("comm_radius", 15.0);
        k_pos_ = this->declare_parameter<double>("k_pos",   0.15);
        k_vel_ = this->declare_parameter<double>("k_vel",   0.10);
        k_goal_ = this->declare_parameter<double>("k_goal",  0.70);
        k_damp_ = this->declare_parameter<double>("k_damp",  0.50);
        k_repel_ = this->declare_parameter<double>("k_repel", 1.80);
    }

protected:
    void compute_velocities() override {
        const double comm_sq = comm_radius_ * comm_radius_;
        const double safe_sq = safe_separation_ * safe_separation_;
        const double norm_factor = (num_drones_ > 1) ? (1.0 / static_cast<double>(num_drones_ - 1)) : 1.0;

        for (int i = 0; i < num_drones_; ++i) {
            Vec3 pos_consensus{0.0, 0.0, 0.0};
            Vec3 vel_consensus{0.0, 0.0, 0.0};
            Vec3 u_repel{0.0, 0.0, 0.0};

            for (int j = 0; j < num_drones_; ++j) {
                if (i == j || !drones_[j].has_odom) continue;

                Vec3 diff = drones_[j].global_pos - drones_[i].global_pos;
                double d_sq = diff.length_sq();

                if (d_sq <= comm_sq) {
                    pos_consensus += diff;
                    vel_consensus += (drones_[j].vel - drones_[i].vel);
                }

                double d = std::sqrt(d_sq);
                if (d_sq < safe_sq && d_sq > 1e-6) {
                    u_repel -= (diff / d) * (k_repel_ * (safe_separation_ - d) / d);
                }
            }

            Vec3 u_consensus = (pos_consensus * k_pos_ + vel_consensus * k_vel_) * norm_factor - drones_[i].vel * k_damp_;
            Vec3 u_goal = (active_goal_ - drones_[i].global_pos) * k_goal_;
            Vec3 bound_force = compute_wall_force(drones_[i].global_pos);

            drones_[i].cmd_vel = apply_safety_limits(
                drones_[i].global_pos,
                u_consensus + u_goal + u_repel + bound_force
            );
        }

        check_and_report_goal_reached(compute_centroid());
    }

private:
    double comm_radius_, k_pos_, k_vel_, k_goal_, k_damp_, k_repel_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4ConsensusGoalController>(argc, argv);
}
