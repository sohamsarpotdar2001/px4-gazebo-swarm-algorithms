#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

struct Waypoint {
    double x;
    double y;
    double z;
};

class WaypointMissionSender : public rclcpp::Node {
public:
    WaypointMissionSender() : Node("waypoint_mission_sender") {
        loop_mission_  = this->declare_parameter<bool>("loop_mission", false);
        dwell_seconds_ = this->declare_parameter<double>("dwell_seconds", 1.5);

        // Generate 15 sequential NED waypoints inside a 6m x 2m x 6m bounding box
        // Centered at (0.0, 0.0, -5.0) with safety margin:
        // x in [-2.5, 2.5] (6m span), y in [-0.8, 0.8] (2m span), z in [-7.5, -2.5] (6m span)
        constexpr int kNumWaypoints = 15;
        waypoints_.reserve(kNumWaypoints);
        for (int i = 0; i < kNumWaypoints; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(kNumWaypoints - 1); // [0.0, 1.0]
            double angle = 2.0 * M_PI * 2.0 * t; // 2 full helical/figure-8 sweeps across the box

            double x = 2.5 * std::cos(angle);
            double y = 0.8 * std::sin(2.0 * angle);
            double z = -2.5 - 5.0 * t; // Descends smoothly from -2.5 m to -7.5 m NED

            waypoints_.push_back({x, y, z});
        }

        goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal", 10);
        mission_comp_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission_comp", 10);

        // Subscribe to /drone_count exclusively (transient_local matches controller publisher)
        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        drone_count_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/drone_count", latched_qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data <= 0) {
                    return;
                }
                if (!has_drone_count_ || msg->data != num_drones_) {
                    num_drones_ = msg->data;
                    has_drone_count_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Updated active swarm size from /drone_count -> N=%d drones.",
                                num_drones_);
                }
            });

        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&WaypointMissionSender::on_goal_reached, this, std::placeholders::_1));

        mission_rec_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission_rec", 10,
            std::bind(&WaypointMissionSender::on_mission_received, this, std::placeholders::_1));

        // 2 Hz state machine timer (500 ms)
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointMissionSender::mission_tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "Waypoint Mission Sender initialized with %zu waypoints inside 6m x 2m x 6m box. Waiting for /drone_count...",
                    waypoints_.size());
    }

private:
    void on_mission_received(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }

        current_idx_ = 0;
        mission_started_ = false;
        waiting_for_ack_ = false;
        in_dwell_ = false;
        mission_complete_ = false;
        logged_wait_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "Received /mission_rec = true! Restarting %zu-waypoint mission for N=%d drones...",
                    waypoints_.size(), num_drones_);
    }

    void publish_current_waypoint() {
        if (current_idx_ >= waypoints_.size()) return;

        const auto& wp = waypoints_[current_idx_];
        geometry_msgs::msg::Point msg{};
        msg.x = wp.x;
        msg.y = wp.y;
        msg.z = wp.z;

        goal_pub_->publish(msg);
        last_pub_time_ = this->now();
        waiting_for_ack_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "[N=%d | Waypoint %zu/%zu] Sent /goal -> (%.2f, %.2f, %.2f)",
                    num_drones_, current_idx_ + 1, waypoints_.size(), wp.x, wp.y, wp.z);
    }

    void on_goal_reached(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !waiting_for_ack_ || mission_complete_) {
            return;
        }

        waiting_for_ack_ = false;
        in_dwell_ = true;
        reached_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "[N=%d | Waypoint %zu/%zu] Reached!",
                    num_drones_, current_idx_ + 1, waypoints_.size());
    }

    void mission_tick() {
        if (mission_complete_ || !has_drone_count_ || num_drones_ <= 0) return;

        // Wait until the swarm controller subscribes to /goal
        if (goal_pub_->get_subscription_count() == 0) {
            if (!logged_wait_) {
                RCLCPP_INFO(this->get_logger(), "Waiting for controller node to subscribe to /goal...");
                logged_wait_ = true;
            }
            return;
        }
        logged_wait_ = false;

        // 1. Send initial waypoint once connected
        if (!mission_started_) {
            mission_started_ = true;
            publish_current_waypoint();
            return;
        }

        // 2. If dwelling at a reached waypoint, advance after dwell_seconds_
        if (in_dwell_) {
            if ((this->now() - reached_time_).seconds() >= dwell_seconds_) {
                in_dwell_ = false;
                ++current_idx_;

                if (current_idx_ >= waypoints_.size()) {
                    if (loop_mission_) {
                        current_idx_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Restarting waypoint mission loop for N=%d drones...", num_drones_);
                        publish_current_waypoint();
                    } else {
                        mission_complete_ = true;

                        std_msgs::msg::Bool comp_msg{};
                        comp_msg.data = true;
                        mission_comp_pub_->publish(comp_msg);

                        RCLCPP_INFO(this->get_logger(),
                                    "[N=%d] All %zu waypoints completed! Published /mission_comp = true. Waiting for /mission_rec...",
                                    num_drones_, waypoints_.size());
                    }
                } else {
                    publish_current_waypoint();
                }
            }
            return;
        }

        // 3. Re-publish current waypoint every 3.0s if still en route (prevents dropped first message)
        if (waiting_for_ack_ && (this->now() - last_pub_time_).seconds() >= 3.0) {
            const auto& wp = waypoints_[current_idx_];
            geometry_msgs::msg::Point msg{};
            msg.x = wp.x;
            msg.y = wp.y;
            msg.z = wp.z;
            goal_pub_->publish(msg);
            last_pub_time_ = this->now();
        }
    }

    int num_drones_ = 0;
    bool has_drone_count_ = false;

    std::vector<Waypoint> waypoints_;
    size_t current_idx_ = 0;
    bool loop_mission_ = false;
    double dwell_seconds_ = 1.5;

    bool logged_wait_ = false;
    bool mission_started_ = false;
    bool waiting_for_ack_ = false;
    bool in_dwell_ = false;
    bool mission_complete_ = false;

    rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time reached_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_comp_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drone_count_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_rec_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointMissionSender>());
    rclcpp::shutdown();
    return 0;
}
