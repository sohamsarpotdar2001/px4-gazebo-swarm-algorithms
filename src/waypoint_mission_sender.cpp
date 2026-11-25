#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
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
        loop_mission_ = this->declare_parameter<bool>("loop_mission", false);
        dwell_seconds_ = this->declare_parameter<double>("dwell_seconds", 1.5);
        waypoints_ = {
            { 3.0,  1.0, -5.0},
            { 3.0, -1.0, -6.0},
            {-3.0, -1.0, -6.0},
            {-3.0,  1.0, -4.0},
            { 0.0,  0.0, -5.0}
        };

        goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal", 10);

        goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&WaypointMissionSender::on_goal_reached, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointMissionSender::mission_tick, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint Mission Sender initialized with %zu waypoints.", waypoints_.size());
    }

private:
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
                    "[Waypoint %zu/%zu] Sent /goal -> (%.2f, %.2f, %.2f)",
                    current_idx_ + 1, waypoints_.size(), wp.x, wp.y, wp.z);
    }

    void on_goal_reached(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !waiting_for_ack_ || mission_complete_) {
            return;
        }

        waiting_for_ack_ = false;
        in_dwell_ = true;
        reached_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "[Waypoint %zu/%zu] Reached! Holding for %.1f s...",
                    current_idx_ + 1, waypoints_.size(), dwell_seconds_);
    }

    void mission_tick() {
        if (mission_complete_) return;

        // Wait until the consensus controller subscribes to /goal
        if (goal_pub_->get_subscription_count() == 0) {
            if (!logged_wait_) {
                RCLCPP_INFO(this->get_logger(), "Waiting for controller node to subscribe to /goal...");
                logged_wait_ = true;
            }
            return;
        }
        logged_wait_ = false;

        // send initial waypoint once connected
        if (!mission_started_) {
            mission_started_ = true;
            publish_current_waypoint();
            return;
        }

        // advance after dwell_seconds_
        if (in_dwell_) {
            if ((this->now() - reached_time_).seconds() >= dwell_seconds_) {
                in_dwell_ = false;
                ++current_idx_;

                if (current_idx_ >= waypoints_.size()) {
                    if (loop_mission_) {
                        current_idx_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Restarting waypoint mission loop...");
                        publish_current_waypoint();
                    } else {
                        mission_complete_ = true;
                        RCLCPP_INFO(this->get_logger(), "All waypoints completed! Mission finished.");
                        rclcpp::shutdown();
                    }
                } else {
                    publish_current_waypoint();
                }
            }
            return;
        }

        // publish current waypoint every 3.0s
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

    std::vector<Waypoint> waypoints_;
    size_t current_idx_ = 0;
    bool loop_mission_ = false;
    double dwell_seconds_ = 0;

    bool logged_wait_ = false;
    bool mission_started_ = false;
    bool waiting_for_ack_ = false;
    bool in_dwell_ = false;
    bool mission_complete_ = false;

    rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time reached_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointMissionSender>());
    rclcpp::shutdown();
    return 0;
}
