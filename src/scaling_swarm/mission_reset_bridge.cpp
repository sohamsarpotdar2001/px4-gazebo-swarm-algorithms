#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <memory>

class MissionResetWatcher : public rclcpp::Node {
public:
    MissionResetWatcher() : Node("mission_reset_watcher") {
        reset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/reset", 10);

        // transient_local QoS matches the latched /drone_count publisher
        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        drone_count_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/drone_count", latched_qos,
            std::bind(&MissionResetWatcher::on_drone_count, this, std::placeholders::_1));

        mission_comp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission_comp", 10,
            std::bind(&MissionResetWatcher::on_mission_comp, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Mission Reset Watcher started. Listening to /drone_count and /mission_comp...");
    }

private:
    void on_drone_count(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data <= 0) {
            return;
        }

        if (!has_initial_count_) {
            prev_drone_count_ = msg->data;
            current_drone_count_ = msg->data;
            has_initial_count_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial drone_count = %d", current_drone_count_);
            return;
        }

        current_drone_count_ = msg->data;
        RCLCPP_INFO(this->get_logger(),
                    "Received /drone_count = %d (previous = %d)",
                    current_drone_count_, prev_drone_count_);

        try_trigger_reset();
    }

    void on_mission_comp(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }

        mission_completed_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Received /mission_comp = true (current_count = %d, prev_count = %d)",
                    current_drone_count_, prev_drone_count_);

        try_trigger_reset();
    }

    void try_trigger_reset() {
        if (!has_initial_count_ || !mission_completed_) {
            return;
        }

        if (current_drone_count_ != prev_drone_count_) {
            std_msgs::msg::Bool reset_msg{};
            reset_msg.data = true;
            reset_pub_->publish(reset_msg);

            RCLCPP_INFO(this->get_logger(),
                        "drone_count changed (%d -> %d) after mission completion! Published /reset = true.",
                        prev_drone_count_, current_drone_count_);

            prev_drone_count_ = current_drone_count_;
            mission_completed_ = false;
        }
    }

    int prev_drone_count_ = 0;
    int current_drone_count_ = 0;
    bool has_initial_count_ = false;
    bool mission_completed_ = false;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drone_count_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_comp_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionResetWatcher>());
    rclcpp::shutdown();
    return 0;
}
