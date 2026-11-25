#include <drone_swarm/algo_base.hpp>

struct DroneAgent : public BaseDroneAgent {
    Vec3 base_offset_ned;
};

class Px4FormationController : public SwarmBaseController<DroneAgent> {
public:
    explicit Px4FormationController(int cli_drone_count = -1) : SwarmBaseController<DroneAgent>("px4_formation_controller", "formation_control", cli_drone_count) {
        triangle_radius_ = this->declare_parameter<double>("triangle_radius", 2.5);
        k_consensus_ = this->declare_parameter<double>("k_consensus", 0.65);
        k_leader_ = this->declare_parameter<double>("k_leader", 0.85);
        k_damp_ = this->declare_parameter<double>("k_damp", 0.35);
        k_repel_ = this->declare_parameter<double>("k_repel", 1.80);

        auto offsets = generate_triangle_offsets(num_drones_, triangle_radius_);

        for (int i = 0; i < num_drones_; ++i) {
            drones_[i].base_offset_ned  = offsets[i];
        }
    }

protected:
    static std::vector<Vec3> generate_triangle_offsets(int n, double radius) {
        std::vector<Vec3> offsets;
        offsets.reserve(n);
        if (n <= 0) return offsets;

        Vec3 vertices[3];
        for (int k = 0; k < 3; ++k) {
            double angle = (2.0 * M_PI * k) / 3.0;
            vertices[k] = {radius * std::cos(angle), radius * std::sin(angle), 0.0};
        }

        if (n <= 3) {
            for (int i = 0; i < n; ++i) offsets.push_back(vertices[i]);
            return offsets;
        }

        for (int i = 0; i < n; ++i) {
            double u = (3.0 * static_cast<double>(i)) / static_cast<double>(n);
            int edge = std::min(2, static_cast<int>(u));
            double t = u - static_cast<double>(edge);
            offsets.push_back(vertices[edge] * (1.0 - t) + vertices[(edge + 1) % 3] * t);
        }
        return offsets;
    }

    void compute_velocities() override {
        double x_center = 0.5 * (num_drones_ - 1) * 3.0;
        double y_center = 0.0;

        double t = static_cast<double>(tick_count_ - 140) * 0.05;

        const double orbit_r = 1.6;
        const double omega   = 0.25;
        Vec3 leader_pos{
            x_center + orbit_r * std::cos(omega * t),
            y_center + orbit_r * std::sin(omega * t),
            target_altitude_ned_ + 0.8 * std::sin(0.35 * t)
        };
        Vec3 leader_vel{
            -orbit_r * omega * std::sin(omega * t),
             orbit_r * omega * std::cos(omega * t),
             0.8 * 0.35 * std::cos(0.35 * t)
        };

        double yaw = 0.20 * t;
        double cy = std::cos(yaw), sy = std::sin(yaw);
        auto rotate_z = [&](const Vec3& v) -> Vec3 {
            return {v.x * cy - v.y * sy, v.x * sy + v.y * cy, v.z};
        };

        const double safe_sq = safe_separation_ * safe_separation_;

        for (int i = 0; i < num_drones_; ++i) {
            Vec3 r_i = rotate_z(drones_[i].base_offset_ned);
            Vec3 u_consensus{0.0, 0.0, 0.0};
            Vec3 u_repel{0.0, 0.0, 0.0};

            for (int j = 0; j < num_drones_; ++j) {
                if (i == j || !drones_[j].has_odom) continue;

                Vec3 r_j = rotate_z(drones_[j].base_offset_ned);

                Vec3 rel_pos_err = (drones_[j].global_pos - drones_[i].global_pos) - (r_j - r_i);
                Vec3 rel_vel_err = (drones_[j].vel - drones_[i].vel);
                u_consensus += rel_pos_err * k_consensus_ + rel_vel_err * k_damp_;

                Vec3 diff = drones_[i].global_pos - drones_[j].global_pos;
                double d_sq = diff.length_sq();
                if (d_sq < safe_sq && d_sq > 1e-6) {
                    double d = std::sqrt(d_sq);
                    u_repel += (diff / d) * (k_repel_ * (safe_separation_ - d) / d);
                }
            }
            if (num_drones_ > 1) {
                u_consensus = u_consensus / static_cast<double>(num_drones_ - 1);
            }

            Vec3 target_i = leader_pos + r_i;
            Vec3 u_leader = (target_i - drones_[i].global_pos) * k_leader_ + leader_vel;

            Vec3 bound_force = compute_wall_force(drones_[i].global_pos);

            drones_[i].cmd_vel = apply_safety_limits(
                drones_[i].global_pos,
                u_consensus + u_leader + u_repel + bound_force
            );
        }

        check_and_report_goal_reached(compute_centroid());
    }

private:
    double triangle_radius_, k_consensus_, k_leader_, k_damp_, k_repel_;
};

int main(int argc, char* argv[]) {
    return run_swarm_node<Px4FormationController>(argc, argv);
}
