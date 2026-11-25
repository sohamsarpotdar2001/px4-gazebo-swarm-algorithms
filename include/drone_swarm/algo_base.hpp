#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double& operator[](int idx) { return (idx == 0) ? x : (idx == 1 ? y : z); }
    double operator[](int idx) const { return (idx == 0) ? x : (idx == 1 ? y : z); }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double length_sq() const { return x * x + y * y + z * z; }
    double length() const { return std::sqrt(length_sq()); }

    Vec3 limited(double max_val) const {
        double len = length();
        return (len > max_val && len > 1e-8) ? (*this * (max_val / len)) : *this;
    }
};

struct BaseDroneAgent {
    int instance_id;
    Vec3 spawn_offset_ned;
    Vec3 local_pos{0.0, 0.0, 0.0};
    Vec3 global_pos{0.0, 0.0, 0.0};
    Vec3 vel{0.0, 0.0, 0.0};
    Vec3 cmd_vel{0.0, 0.0, 0.0};

    bool has_odom = false;
    uint8_t nav_state = 0;
    uint8_t arming_state = 0;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub;
};

template<typename AgentT = BaseDroneAgent>

class SwarmBaseController : public rclcpp::Node {
    static_assert(std::is_base_of<BaseDroneAgent, AgentT>::value, "AgentT must inherit from BaseDroneAgent");
public:
    SwarmBaseController(const std::string& node_name, const std::string& algo_name, int cli_drone_count = -1) : Node(node_name), algo_name_(algo_name) {
        int default_count = 3;
        if (const char* env_p = std::getenv("PX4_DRONE_COUNT")) {
            int parsed = std::atoi(env_p);
            if (parsed > 0) default_count = parsed;
        }
        if (cli_drone_count > 0) {
            default_count = cli_drone_count;
        }

        num_drones_ = this->declare_parameter<int>("drone_count", default_count);
        if (cli_drone_count > 0) {
            num_drones_ = cli_drone_count;
        }

        target_altitude_ned_ = this->declare_parameter<double>("target_altitude_ned", -5.0);
        box_size_            = this->declare_parameter<double>("box_size", 44.0);
        safe_separation_     = this->declare_parameter<double>("safe_separation", 1.6);
        max_speed_           = this->declare_parameter<double>("max_speed", 1.2);
        goal_tolerance_      = this->declare_parameter<double>("goal_tolerance", 0.45);

        active_goal_ = {0.0, 0.0, target_altitude_ned_};

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/goal", 10, std::bind(&SwarmBaseController::goal_subscriber_cb, this, std::placeholders::_1));

        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        drone_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/drone_count", latched_qos);
        algo_pub_ = this->create_publisher<std_msgs::msg::String>("/algo", latched_qos);
        publish_drone_count();
        publish_algo();

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        RCLCPP_INFO(this->get_logger(), "Initializing %s Controller for %d drones", algo_name_.c_str(), num_drones_);

        drones_.resize(num_drones_);
        for (int i = 0; i < num_drones_; ++i) {
            auto& d = drones_[i];
            d.instance_id = i + 1;

            double sign = (i % 2 == 0) ? 1.0 : -1.0;
            double y_enu = sign * static_cast<double>(i);
            d.spawn_offset_ned = {y_enu, y_enu, 0.0};

            std::string ns = "/uav_" + std::to_string(d.instance_id);

            d.offboard_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                ns + "/fmu/in/offboard_control_mode", 10);
            d.setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                ns + "/fmu/in/trajectory_setpoint", 10);
            d.cmd_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
                ns + "/fmu/in/vehicle_command", 10);

            d.pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                ns + "/fmu/out/vehicle_local_position_v1", sensor_qos,
                [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    drones_[i].local_pos = {msg->x, msg->y, msg->z};
                    drones_[i].global_pos = drones_[i].local_pos + drones_[i].spawn_offset_ned;
                    drones_[i].vel = {msg->vx, msg->vy, msg->vz};
                    drones_[i].has_odom = true;
                });

            d.status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                ns + "/fmu/out/vehicle_status_v4", sensor_qos,
                [this, i](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                    drones_[i].nav_state = msg->nav_state;
                    drones_[i].arming_state = msg->arming_state;
                });

            RCLCPP_INFO(this->get_logger(), "Registered Drone %d on %s | Spawn NED: (%.1f, %.1f)",
                        d.instance_id, ns.c_str(), d.spawn_offset_ned.x, d.spawn_offset_ned.y);
        }

        timer_ = this->create_wall_timer(50ms, std::bind(&SwarmBaseController::control_loop, this));
    }

    virtual ~SwarmBaseController() = default;

protected:
    virtual void compute_velocities() = 0;

    virtual void reset() { }

    virtual void goal_subscriber_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        const double half_box = box_size_ * 0.5;
        const double margin = 0.5;

        active_goal_.x = std::clamp(msg->x, -half_box + margin, half_box - margin);
        active_goal_.y = std::clamp(msg->y, -half_box + margin, half_box - margin);
        active_goal_.z = std::clamp(msg->z, -6.0 - half_box + margin, -6.0 + half_box - margin);

        if (!has_external_goal_) {
            RCLCPP_INFO(this->get_logger(), "First /goal received! Enabling offboard control sequence.");
        }

        has_external_goal_ = true;
        goal_reached_reported_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "Received new /goal waypoint (clamped to %.1fm box NED): (%.2f, %.2f, %.2f)",
                    box_size_, active_goal_.x, active_goal_.y, active_goal_.z);
    }

    Vec3 compute_centroid() const {
        Vec3 centroid{0.0, 0.0, 0.0};
        if (num_drones_ == 0) return centroid;
        for (const auto& d : drones_) {
            centroid += d.global_pos;
        }
        return centroid / static_cast<double>(num_drones_);
    }

    Vec3 compute_wall_force(const Vec3& pos, double wall_margin = 1.2, double wall_gain = 1.5) const {
        const double half_box = box_size_ * 0.5;
        const double x_min = -half_box, x_max = half_box;
        const double y_min = -half_box, y_max = half_box;
        const double z_min = -6.0 - half_box, z_max = -6.0 + half_box;

        Vec3 force{0.0, 0.0, 0.0};
        if (pos.x < x_min + wall_margin) force.x += wall_gain * ((x_min + wall_margin) - pos.x);
        if (pos.x > x_max - wall_margin) force.x -= wall_gain * (pos.x - (x_max - wall_margin));
        if (pos.y < y_min + wall_margin) force.y += wall_gain * ((y_min + wall_margin) - pos.y);
        if (pos.y > y_max - wall_margin) force.y -= wall_gain * (pos.y - (y_max - wall_margin));
        if (pos.z < z_min + wall_margin) force.z += wall_gain * ((z_min + wall_margin) - pos.z);
        if (pos.z > z_max - wall_margin) force.z -= wall_gain * (pos.z - (z_max - wall_margin));
        return force;
    }

    Vec3 apply_safety_limits(const Vec3& pos, Vec3 v_cmd) {
        const double half_box = box_size_ * 0.5;
        const double x_min = -half_box, x_max = half_box;
        const double y_min = -half_box, y_max = half_box;
        const double z_min = -6.0 - half_box, z_max = -6.0 + half_box;

        v_cmd = v_cmd.limited(max_speed_);

        if (pos.x <= x_min && v_cmd.x < 0.0) v_cmd.x = 0.0;
        if (pos.x >= x_max && v_cmd.x > 0.0) v_cmd.x = 0.0;
        if (pos.y <= y_min && v_cmd.y < 0.0) v_cmd.y = 0.0;
        if (pos.y >= y_max && v_cmd.y > 0.0) v_cmd.y = 0.0;
        if (pos.z <= z_min && v_cmd.z < 0.0) v_cmd.z = 0.0;
        if (pos.z >= z_max && v_cmd.z > 0.0) v_cmd.z = 0.0;

        return v_cmd;
    }

    void check_and_report_goal_reached(const Vec3& reference_point) {
        double goal_error = (reference_point - active_goal_).length();
        if (has_external_goal_ && !goal_reached_reported_ && goal_error <= goal_tolerance_) {
            goal_reached_reported_ = true;

            std_msgs::msg::Bool reached_msg{};
            reached_msg.data = true;
            goal_reached_pub_->publish(reached_msg);

            RCLCPP_INFO(this->get_logger(),
                        "GOAL REACHED! Position (%.2f, %.2f, %.2f) within %.2f m of /goal (%.2f, %.2f, %.2f)",
                        reference_point.x, reference_point.y, reference_point.z, goal_error, active_goal_.x, active_goal_.y, active_goal_.z);
        }
    }

    int num_drones_ = 3;
    double target_altitude_ned_ = -5.0;
    double box_size_ = 44.0;
    double safe_separation_ = 1.6;
    double max_speed_ = 1.2;
    double goal_tolerance_ = 0.45;


    uint64_t idle_tick_count_ = 0;
    uint64_t tick_count_ = 0;

    Vec3 active_goal_{0.0, 0.0, target_altitude_ned_};
    bool has_external_goal_ = false;
    bool goal_reached_reported_ = false;
    std::vector<AgentT> drones_;

private:
    void publish_drone_count() {
        std_msgs::msg::Int32 count_msg{};
        count_msg.data = num_drones_;
        drone_count_pub_->publish(count_msg);
    }

    void publish_algo() {
        std_msgs::msg::String msg{};
        msg.data = algo_name_;
        algo_pub_->publish(msg);
    }

    uint64_t now_us() {
        return this->get_clock()->now().nanoseconds() / 1000ULL;
    }

    void send_vehicle_command(BaseDroneAgent& d, uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now_us();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = static_cast<uint8_t>(d.instance_id + 1);
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        d.cmd_pub->publish(msg);
    }

    void publish_offboard_mode(BaseDroneAgent& d, bool position_control, bool velocity_control) {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now_us();
        msg.position = position_control;
        msg.velocity = velocity_control;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        d.offboard_pub->publish(msg);
    }

    void control_loop() {
        publish_algo();
        ++idle_tick_count_;
        if (idle_tick_count_ % 20 == 0) {
            publish_drone_count();
        }

        for (const auto& d : drones_) {
            if (!d.has_odom) {
                if (idle_tick_count_ % 40 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for odometry from /uav_%d...", d.instance_id);
                }
                return;
            }
        }

        if (!has_external_goal_) {
            if (idle_tick_count_ % 40 == 0) {
                RCLCPP_INFO(this->get_logger(), "Odometry ready. Waiting for /goal before sending offboard commands...");
            }
            return;
        }

        ++tick_count_;

        bool takeoff_complete = (tick_count_ > 140);
        if (!takeoff_complete) {
            reset();
        } else {
            compute_velocities();
        }

        for (int i = 0; i < num_drones_; ++i) {
            auto& d = drones_[i];

            if (tick_count_ == 40) {
                send_vehicle_command(d, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
                send_vehicle_command(d, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                RCLCPP_INFO(this->get_logger(), "Sent Offboard + Arm command to Drone %d", d.instance_id);
            }

            px4_msgs::msg::TrajectorySetpoint sp{};
            sp.timestamp = now_us();

            if (!takeoff_complete) {
                publish_offboard_mode(d, true, false);
                sp.position = {0.0f, 0.0f, static_cast<float>(target_altitude_ned_)};
                sp.yaw = 0.0f;
            } else {
                publish_offboard_mode(d, false, true);
                sp.position = {NAN, NAN, NAN};
                sp.velocity = {
                    static_cast<float>(d.cmd_vel.x),
                    static_cast<float>(d.cmd_vel.y),
                    static_cast<float>(d.cmd_vel.z)
                };

                if (std::hypot(d.cmd_vel.x, d.cmd_vel.y) > 0.12) {
                    sp.yaw = static_cast<float>(std::atan2(d.cmd_vel.y, d.cmd_vel.x));
                } else {
                    sp.yaw = NAN;
                }
            }

            d.setpoint_pub->publish(sp);
        }
    }

    std::string algo_name_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr algo_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drone_count_pub_;
};

// Reusable CLI parser + ROS 2 spin helper
template <typename ControllerT>
int run_swarm_node(int argc, char* argv[]) {
    std::vector<std::string> non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    int cli_drone_count = -1;
    for (size_t i = 1; i < non_ros_args.size(); ++i) {
        const std::string& arg = non_ros_args[i];
        if ((arg == "--drone_count" || arg == "-n") && i + 1 < non_ros_args.size()) {
            cli_drone_count = std::atoi(non_ros_args[++i].c_str());
        } else if (arg.rfind("--drone_count=", 0) == 0) {
            cli_drone_count = std::atoi(arg.substr(14).c_str());
        } else {
            try {
                int val = std::stoi(arg);
                if (val > 0) cli_drone_count = val;
            } catch (...) {}
        }
    }

    rclcpp::spin(std::make_shared<ControllerT>(cli_drone_count));
    rclcpp::shutdown();
    return 0;
}
