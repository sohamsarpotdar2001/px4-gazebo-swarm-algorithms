#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};

struct DroneTelemetry {
    int instance_id = 1;
    Vec3 spawn_offset_ned{0.0, 0.0, 0.0};
    Vec3 global_pos{0.0, 0.0, 0.0};
    bool has_odom = false;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub;
};

struct TimeSeriesSample {
    double t_sec;
    double tracking_error;
    double min_separation;
    int active_sep_violations;
    int cumulative_sep_violations;
    int cumulative_collisions;
    double collision_rate_hz;
};

struct GoalCompletionRecord {
    int waypoint_idx;
    double reach_duration_sec;
    double settling_time_sec;
    Vec3 goal_ned;
};

class SwarmMetricsLogger : public rclcpp::Node {
public:
    explicit SwarmMetricsLogger(int cli_drone_count = -1, const std::string& algo_label = "boids")
        : Node("swarm_metrics_logger"), algo_label_(algo_label) {
        int default_count = 3;
        if (const char* env_p = std::getenv("PX4_DRONE_COUNT")) {
            int parsed = std::atoi(env_p);
            if (parsed > 0) default_count = parsed;
        }
        if (cli_drone_count > 0) default_count = cli_drone_count;

        num_drones_          = this->declare_parameter<int>("drone_count", default_count);
        if (cli_drone_count > 0) num_drones_ = cli_drone_count;

        safe_separation_     = this->declare_parameter<double>("safe_separation", 1.6);
        collision_radius_    = this->declare_parameter<double>("collision_radius", 0.65);
        settling_tolerance_  = this->declare_parameter<double>("settling_tolerance", 0.45);
        output_prefix_       = this->declare_parameter<std::string>("output_prefix", "swarm_" + algo_label_);

        active_goal_ = {0.0, 0.0, -5.0};

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        drones_.resize(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            auto& d = drones_[i];
            d.instance_id = i + 1;

            double sign = (i % 2 == 0) ? 1.0 : -1.0;
            double y_enu = sign * static_cast<double>(i * 2);
            d.spawn_offset_ned = {y_enu, 0.0, 0.0};

            std::string topic = "/uav_" + std::to_string(d.instance_id) + "/fmu/out/vehicle_local_position_v1";
            d.pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                topic, sensor_qos,
                [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    Vec3 local{msg->x, msg->y, msg->z};
                    drones_[i].global_pos = local + drones_[i].spawn_offset_ned;
                    drones_[i].has_odom = true;
                });
        }

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/goal", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                Vec3 incoming_goal{msg->x, msg->y, msg->z};

                if (has_goal_ && (incoming_goal - active_goal_).length() < 1e-4) {
                    return;
                }

                active_goal_ = incoming_goal;
                has_goal_ = true;
                waiting_for_goal_reach_ = true;
                ++goal_counter_;
                current_goal_start_time_ = this->now();
                last_outside_tolerance_time_ = current_goal_start_time_;
                has_entered_tolerance_ = false;

                RCLCPP_INFO(this->get_logger(),
                            "[Goal #%d] Tracking new target (%.2f, %.2f, %.2f)",
                            goal_counter_, active_goal_.x, active_goal_.y, active_goal_.z);
            });

        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && waiting_for_goal_reach_) {
                    waiting_for_goal_reach_ = false;
                    rclcpp::Time now_t = this->now();
                    double elapsed = (now_t - current_goal_start_time_).seconds();
                    double settling_sec = has_entered_tolerance_
                        ? std::max(0.0, (last_outside_tolerance_time_ - current_goal_start_time_).seconds())
                        : elapsed;

                    goal_records_.push_back({
                        goal_counter_,
                        elapsed,
                        settling_sec,
                        active_goal_
                    });

                    RCLCPP_INFO(this->get_logger(),
                                "[Goal #%d] Reached in %.2f s (settling time: %.2f s)",
                                goal_counter_, elapsed, settling_sec);
                }
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&SwarmMetricsLogger::sample_metrics, this));

        RCLCPP_INFO(this->get_logger(),
                    "Swarm Metrics Logger started for %d drones (algo='%s'). Press Ctrl+C when done to generate Gnuplot graphs.",
                    num_drones_, algo_label_.c_str());
    }

    void export_and_plot() {
        if (samples_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No telemetry samples collected; skipping Gnuplot generation.");
            return;
        }

        std::string ts_csv   = output_prefix_ + "_timeseries.csv";
        std::string goal_csv = output_prefix_ + "_goals.csv";
        std::string gp_file  = output_prefix_ + "_plot.gp";
        std::string png_file = output_prefix_ + "_report.png";

        {
            std::ofstream out(ts_csv);
            out << "time_s,tracking_error_m,min_separation_m,active_sep_violations,cum_sep_violations,cum_collisions,collision_rate_hz\n";
            for (const auto& s : samples_) {
                out << std::fixed << std::setprecision(4)
                    << s.t_sec << ","
                    << s.tracking_error << ","
                    << s.min_separation << ","
                    << s.active_sep_violations << ","
                    << s.cumulative_sep_violations << ","
                    << s.cumulative_collisions << ","
                    << s.collision_rate_hz << "\n";
            }
        }

        {
            std::ofstream out(goal_csv);
            out << "waypoint_idx,reach_time_s,settling_time_s,goal_x,goal_y,goal_z\n";
            if (goal_records_.empty()) {
                out << "0,0.0,0.0,0.0,0.0,-5.0\n";
            } else {
                for (const auto& g : goal_records_) {
                    out << g.waypoint_idx << ","
                        << std::fixed << std::setprecision(4)
                        << g.reach_duration_sec << ","
                        << g.settling_time_sec << ","
                        << g.goal_ned.x << "," << g.goal_ned.y << "," << g.goal_ned.z << "\n";
                }
            }
        }

        {
            std::ofstream gp(gp_file);
            gp << "set terminal pngcairo size 1600,1000 enhanced font 'Sans,11'\n";
            gp << "set output '" << png_file << "'\n";
            gp << "set datafile separator ','\n";
            gp << "set grid lw 1 lc rgb '#dddddd'\n";
            gp << "set multiplot layout 2,2 title 'Multi-Drone Swarm Performance Metrics ("
               << algo_label_ << ", N=" << num_drones_ << ")' font 'Sans-Bold,14'\n\n";

            // Panel 1: Tracking Error
            gp << "set title 'Centroid Tracking Error vs. Time'\n";
            gp << "set xlabel 'Time (s)'\n";
            gp << "set ylabel 'Tracking Error (m)'\n";
            gp << "plot '" << ts_csv << "' every ::1 using 1:2 with lines lw 2 lc rgb '#1f77b4' title 'Centroid Error (m)'\n\n";

            // Panel 2: Minimum Inter-Drone Separation & Safe Threshold
            gp << "set title 'Minimum Inter-Drone Separation & Violations'\n";
            gp << "set xlabel 'Time (s)'\n";
            gp << "set ylabel 'Distance (m)'\n";
            gp << "plot '" << ts_csv << "' every ::1 using 1:3 with lines lw 2 lc rgb '#2ca02c' title 'Min Pairwise Distance (m)', \\\n";
            gp << "     " << safe_separation_ << " with lines dt 2 lw 2 lc rgb '#d62728' title 'Safe Separation (" << safe_separation_ << " m)'\n\n";

            // Panel 3: Rolling Collision Rate & Cumulative Separation Violations
            gp << "set title 'Cumulative Separation Violations'\n";
            gp << "set xlabel 'Time (s)'\n";
            gp << "set ylabel 'Collision Rate (collisions/s)'\n";
            gp << "set y2label 'Cumulative Separation Violations'\n";
            gp << "set y2tics\n";
            gp << "plot '" << ts_csv << "' every ::1 using 1:7 with lines lw 2 lc rgb '#d62728' axes x1y1 title 'Rolling Collision Rate (Hz)', \\\n";
            gp << "     '" << ts_csv << "' every ::1 using 1:5 with lines lw 2 lc rgb '#ff7f0e' axes x1y2 title 'Cumulative Sep. Violations'\n";
            gp << "unset y2tics\n";
            gp << "unset y2label\n\n";

            // Panel 4: Goal-Reaching Time per Waypoint
            gp << "set title 'Goal-Reaching Time per Waypoint'\n";
            gp << "set xlabel 'Waypoint Index'\n";
            gp << "set ylabel 'Time to Reach Goal (s)'\n";
            gp << "set style fill solid 0.75 border -1\n";
            gp << "set boxwidth 0.5\n";
            gp << "set yrange [0:*]\n";
            gp << "plot '" << goal_csv << "' every ::1 using 1:2:xtic(1) with boxes lc rgb '#9467bd' title 'Goal Reach Duration (s)'\n\n";

            gp << "unset multiplot\n";
        }

        std::string cmd = "gnuplot " + gp_file;
        int ret = std::system(cmd.c_str());

        double avg_err = 0.0;
        for (const auto& s : samples_) avg_err += s.tracking_error;
        avg_err /= static_cast<double>(samples_.size());

        double avg_settling = 0.0;
        for (const auto& g : goal_records_) avg_settling += g.settling_time_sec;
        if (!goal_records_.empty()) {
            avg_settling /= static_cast<double>(goal_records_.size());
        }

        RCLCPP_INFO(this->get_logger(), "================ SWARM METRICS SUMMARY ================");
        RCLCPP_INFO(this->get_logger(), "Samples Recorded           : %zu (%.1f s)", samples_.size(), samples_.back().t_sec);
        RCLCPP_INFO(this->get_logger(), "Mean Tracking Error        : %.3f m", avg_err);
        RCLCPP_INFO(this->get_logger(), "Total Separation Violations: %d", cumulative_sep_violations_);
        RCLCPP_INFO(this->get_logger(), "Total Collisions (<%.2fm)  : %d", collision_radius_, cumulative_collisions_);
        RCLCPP_INFO(this->get_logger(), "Goals Reached              : %zu (mean settling: %.2f s)",
                    goal_records_.size(), avg_settling);
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Gnuplot graph saved to     : %s", png_file.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gnuplot failed (is `gnuplot` installed? Run: sudo apt install gnuplot)");
        }
    }

private:
    void sample_metrics() {
        for (const auto& d : drones_) {
            if (!d.has_odom) return;
        }

        rclcpp::Time now_t = this->now();
        if (!started_) {
            started_ = true;
            start_time_ = now_t;
        }
        double t_sec = (now_t - start_time_).seconds();

        Vec3 centroid{0.0, 0.0, 0.0};
        for (const auto& d : drones_) centroid += d.global_pos;
        centroid = centroid / static_cast<double>(num_drones_);
        double tracking_err = (centroid - active_goal_).length();

        if (waiting_for_goal_reach_) {
            if (tracking_err > settling_tolerance_) {
                last_outside_tolerance_time_ = now_t;
            } else {
                has_entered_tolerance_ = true;
            }
        }

        double min_sep = std::numeric_limits<double>::infinity();
        int active_violations = 0;
        int new_collisions_this_step = 0;

        for (int i = 0; i < num_drones_; ++i) {
            for (int j = i + 1; j < num_drones_; ++j) {
                double dist = (drones_[i].global_pos - drones_[j].global_pos).length();
                min_sep = std::min(min_sep, dist);

                if (dist < safe_separation_) {
                    ++active_violations;
                    ++cumulative_sep_violations_;
                }
                if (dist < collision_radius_) {
                    ++new_collisions_this_step;
                    ++cumulative_collisions_;
                }
            }
        }
        if (num_drones_ < 2) min_sep = 0.0;

        // 3. Rolling 2.0-second Collision Rate (collisions/sec)
        for (int c = 0; c < new_collisions_this_step; ++c) {
            recent_collision_timestamps_.push_back(t_sec);
        }
        const double window_sec = 2.0;
        while (!recent_collision_timestamps_.empty() &&
               (t_sec - recent_collision_timestamps_.front()) > window_sec) {
            recent_collision_timestamps_.erase(recent_collision_timestamps_.begin());
        }
        double collision_rate = static_cast<double>(recent_collision_timestamps_.size()) / window_sec;

        samples_.push_back({
            t_sec,
            tracking_err,
            min_sep,
            active_violations,
            cumulative_sep_violations_,
            cumulative_collisions_,
            collision_rate
        });
    }

    int num_drones_ = 3;
    std::string algo_label_ = "boids";
    std::string output_prefix_;

    double safe_separation_    = 1.6;
    double collision_radius_   = 0.65;
    double settling_tolerance_ = 0.45;

    bool started_ = false;
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    Vec3 active_goal_{0.0, 0.0, -5.0};
    bool has_goal_ = false;
    bool waiting_for_goal_reach_ = false;
    bool has_entered_tolerance_ = false;
    int goal_counter_ = 0;
    rclcpp::Time current_goal_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_outside_tolerance_time_{0, 0, RCL_ROS_TIME};

    int cumulative_sep_violations_ = 0;
    int cumulative_collisions_     = 0;
    std::vector<double> recent_collision_timestamps_;

    std::vector<DroneTelemetry> drones_;
    std::vector<TimeSeriesSample> samples_;
    std::vector<GoalCompletionRecord> goal_records_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    std::vector<std::string> non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    int cli_drone_count = -1;
    std::string algo_label = "boids";

    for (size_t i = 1; i < non_ros_args.size(); ++i) {
        const std::string& arg = non_ros_args[i];
        if ((arg == "--drone_count" || arg == "-n") && i + 1 < non_ros_args.size()) {
            cli_drone_count = std::atoi(non_ros_args[++i].c_str());
        } else if ((arg == "--algo" || arg == "-a") && i + 1 < non_ros_args.size()) {
            algo_label = non_ros_args[++i];
        } else {
            try {
                int val = std::stoi(arg);
                if (val > 0) cli_drone_count = val;
            } catch (...) {
                algo_label = arg;
            }
        }
    }

    auto node = std::make_shared<SwarmMetricsLogger>(cli_drone_count, algo_label);
    rclcpp::spin(node);
    node->export_and_plot();
    rclcpp::shutdown();
    return 0;
}
