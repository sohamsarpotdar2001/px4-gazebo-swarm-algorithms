#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class WaypointFollower : public rclcpp::Node
{
public:
  WaypointFollower()
  : Node("waypoint_follower")
  {
    this->declare_parameter<std::vector<double>>(
      "waypoints",
      {0.0, 0.0, -3.0, 5.0, 0.0, -3.0, 5.0, 5.0, -3.0});
    this->declare_parameter<double>("acceptance_radius", 0.5);

    this->declare_parameter<int>("target_system", 2);
    target_system_ = static_cast<uint8_t>(this->get_parameter("target_system").as_int());

    const auto flat_waypoints =
      this->get_parameter("waypoints").as_double_array();
    acceptance_radius_ =
      this->get_parameter("acceptance_radius").as_double();

    if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0) {
      throw std::runtime_error(
        "waypoints must be a non-empty flat array of x,y,z triples");
    }

    for (std::size_t i = 0; i < flat_waypoints.size(); i += 3) {
      waypoints_.push_back({
        static_cast<float>(flat_waypoints[i]),
        static_cast<float>(flat_waypoints[i + 1]),
        static_cast<float>(flat_waypoints[i + 2])});
    }

    const auto px4_output_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .transient_local();

    position_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "fmu/out/vehicle_local_position_v1",
        px4_output_qos,
        [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          position_ = *msg;
          have_position_ = true;
        });

    offboard_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", 10);
    setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", 10);
    command_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&WaypointFollower::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu NED waypoints; waiting for local position.",
      waypoints_.size());
  }

private:
  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
  }

  void publish_vehicle_command(
    uint16_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void control_loop()
  {
    if (!have_position_) {
      RCLCPP_INFO(get_logger(), "Still waiting for local position");
      return;
    }

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp_us();
    mode.position = true;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_pub_->publish(mode);

    const auto & target = waypoints_[waypoint_index_];

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = timestamp_us();
    setpoint.position = target;
    setpoint.velocity = {
      NAN, NAN, NAN};
    setpoint.acceleration = {
      NAN, NAN, NAN};
    setpoint.yaw = NAN;
    setpoint.yawspeed = NAN;
    setpoint_pub_->publish(setpoint);

    ++ticks_;
    if (ticks_ < 20) {
      RCLCPP_INFO(get_logger(), "Publishing setpoints: %u", ticks_);
      return;
    }

    if (!mode_requested_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0F, 6.0F);
      mode_requested_ = true;
      RCLCPP_WARN(get_logger(), "Offboard requested. Ensure setpoints are streamed for >1 second.");
      return;
    }

    if (!arm_requested_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0F);
      arm_requested_ = true;
      RCLCPP_WARN(get_logger(), "Arm command sent; Verify PX4 accepts it.");
      return;
    }

    const double dx = position_.x - target[0];
    const double dy = position_.y - target[1];
    const double dz = position_.z - target[2];
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance <= acceptance_radius_) {
      if (waypoint_index_ + 1 < waypoints_.size()) {
        ++waypoint_index_;
        RCLCPP_INFO(
          this->get_logger(), "Advancing to waypoint %zu",
          waypoint_index_ + 1);
      } else if (!final_waypoint_logged_) {
        RCLCPP_INFO(
          this->get_logger(), "Final waypoint reached; holding position.");
        final_waypoint_logged_ = true;
      }
    }
  }

  std::vector<std::array<float, 3>> waypoints_;
  std::size_t waypoint_index_{0};
  double acceptance_radius_{0.5};
  unsigned int ticks_{0};
  bool have_position_{false};
  bool mode_requested_{false};
  bool arm_requested_{false};
  bool final_waypoint_logged_{false};
  uint8_t target_system_{2};
  px4_msgs::msg::VehicleLocalPosition position_{};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_; 
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  rclcpp::shutdown();
  return 0;
}
