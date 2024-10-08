// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lsdb_interface/lsdb_interface.hpp"

#include <cfloat>

LsdbInterface::LsdbInterface(const rclcpp::NodeOptions & node_options)
: Node("lsdb_interface", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  wheel_tread_ = vehicle_info.wheel_tread_m;
  wheel_radius_ = vehicle_info.wheel_radius_m;
  speed_scale_factor_ = declare_parameter<double>("speed_scale_factor", 1.0);
  loop_rate_ = declare_parameter<double>("loop_rate", 50.0);
  control_cmd_timeout_sec_ = declare_parameter<double>("control_cmd_timeout_sec", 1.0);

  // Subscribe from Autoware
  control_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1,
      std::bind(&LsdbInterface::onAckermannControlCmd, this, _1));
  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&LsdbInterface::onGearCmd, this, _1));
  turn_indicators_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", 1,
      std::bind(&LsdbInterface::onTurnIndicatorsCmd, this, _1));
  hazard_lights_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", 1,
      std::bind(&LsdbInterface::onHazardLightsCmd, this, _1));
  head_lights_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>(
      "/control/command/head_lights_cmd", 1,
      std::bind(&LsdbInterface::onHeadLightsCmd, this, _1));
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1, std::bind(&LsdbInterface::onEmergencyCmd, this, _1));
  // Subscribe from lsdb
  lsdb_left_status_sub_ = this->create_subscription<lsdb_msgs::msg::LsdbStatusStamped>(
    "~/input/lsdb/left/status", 1, std::bind(&LsdbInterface::onLsdbLeftStatus, this, _1));
  lsdb_right_status_sub_ = this->create_subscription<lsdb_msgs::msg::LsdbStatusStamped>(
    "~/input/lsdb/right/status", 1, std::bind(&LsdbInterface::onLsdbRightStatus, this, _1));
  // Subscribe from AVA-3510 Din
  din0_estop_sub_ = this->create_subscription<dio_ros_driver::msg::DIOPort>(
    "/dio/din0", 1, std::bind(&LsdbInterface::onDin0Estop, this, _1));

  // Publish to autoware
  velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1);
  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1);
  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1);
  turn_indicators_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 1);
  hazard_lights_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", 1);
  control_mode_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", 1);
  velocity_kmph_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity_kmph", 1);
  steering_wheel_deg_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/steering_wheel_deg", 1);
  // Publish to s1
  s1_cmd_left_pub_ =
    this->create_publisher<lsdb_msgs::msg::LsdbCommandStamped>("~/output/left/command", 1);
  s1_cmd_right_pub_ =
    this->create_publisher<lsdb_msgs::msg::LsdbCommandStamped>("~/output/right/command", 1);
  // Publish to AVA-3510 Dout
  dout0_brake_light_pub_ = this->create_publisher<dio_ros_driver::msg::DIOPort>("/dio/dout0", 1);
  dout1_front_light_pub_ = this->create_publisher<dio_ros_driver::msg::DIOPort>("/dio/dout1", 1);
  dout2_left_blinker_pub_ = this->create_publisher<dio_ros_driver::msg::DIOPort>("/dio/dout2", 1);
  dout3_right_blinker_pub_ = this->create_publisher<dio_ros_driver::msg::DIOPort>("/dio/dout3", 1);

  setupDiagnosticUpdater();

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  cmd_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&LsdbInterface::onTimer, this));
}

void LsdbInterface::onTimer()
{
  diagnostic_updater_.force_update();

  publishVehicleControlMode();
  if (is_control_command_timeout_) {
    publishZeroCommand();
  } else {
    publishCommand();
  }
}

void LsdbInterface::publishZeroCommand()
{
  dio_ros_driver::msg::DIOPort dout0_msg;
  dout0_msg.value = false;
  dout0_brake_light_pub_->publish(dout0_msg);

  s1_right_cmd_.stamp = this->now();
  s1_left_cmd_.stamp = this->now();
  s1_left_cmd_.command.speed = 0.0;
  s1_right_cmd_.command.speed = 0.0;
  s1_cmd_right_pub_->publish(s1_right_cmd_);
  s1_cmd_left_pub_->publish(s1_left_cmd_);
}

void LsdbInterface::publishCommand()
{
  dio_ros_driver::msg::DIOPort dout0_msg;
  if (s1_right_cmd_.command.speed < FLT_EPSILON && s1_left_cmd_.command.speed < FLT_EPSILON) {
    dout0_msg.value = false;
    dout0_brake_light_pub_->publish(dout0_msg);
  } else {
    dout0_msg.value = true;
    dout0_brake_light_pub_->publish(dout0_msg);
  }

  s1_cmd_right_pub_->publish(s1_right_cmd_);
  s1_cmd_left_pub_->publish(s1_left_cmd_);
}

void LsdbInterface::publishVehicleControlMode()
{
  auto msg = autoware_auto_vehicle_msgs::msg::ControlModeReport{};
  msg.stamp = this->now();
  msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_status_pub_->publish(msg);
}

void LsdbInterface::publishVelocityAndSteering(
  lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr left_msg,
  lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr right_msg)
{
  const double rpm_unit = 2 * wheel_radius_ * M_PI / 60.0;

  // calcVehicleTwist
  autoware_auto_vehicle_msgs::msg::VelocityReport twist;
  twist.header.stamp = left_msg->stamp;
  twist.header.frame_id = "base_link";
  double left_wheel_vel = left_msg->status.motor_speed_rpm * rpm_unit * speed_scale_factor_;
  double right_wheel_vel = right_msg->status.motor_speed_rpm * rpm_unit * speed_scale_factor_;

  // LSDBでは取り付け向きの都合上-1をつける
  left_wheel_vel *= -1.0;

  twist.longitudinal_velocity = (left_wheel_vel + right_wheel_vel) / 2;
  twist.heading_rate = (right_wheel_vel - left_wheel_vel) / wheel_tread_;
  velocity_status_pub_->publish(twist);

  tier4_debug_msgs::msg::Float32Stamped velocity_kmph_msg;
  velocity_kmph_msg.data = twist.longitudinal_velocity * 3.6;
  velocity_kmph_msg.stamp = this->now();
  velocity_kmph_status_pub_->publish(velocity_kmph_msg);

  autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
  steer_msg.stamp = left_msg->stamp;
  steer_msg.steering_tire_angle =
    twist.longitudinal_velocity != 0.0
      ? std::atan(twist.heading_rate * wheel_base_ / twist.longitudinal_velocity)
      : 0.0;
  steering_status_pub_->publish(steer_msg);

  tier4_debug_msgs::msg::Float32Stamped steer_wheel_deg_msg;
  steer_wheel_deg_msg.data = steer_msg.steering_tire_angle * 180.0 / M_PI;
  steer_wheel_deg_msg.stamp = this->now();
  steering_wheel_deg_status_pub_->publish(steer_wheel_deg_msg);
}

void LsdbInterface::onAckermannControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;

  if (!gear_cmd_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "gear command is not subscribed");
    return;
  }

  s1_right_cmd_.stamp = msg->stamp;
  s1_left_cmd_.stamp = msg->stamp;

  double trans_vel = msg->longitudinal.speed;
  double angular_vel =
    msg->longitudinal.speed * std::tan(msg->lateral.steering_tire_angle) / wheel_base_;

  double left_wheel_rpm = 0.0;
  double right_wheel_rpm = 0.0;
  if (!is_emergency_ && gear_cmd_ptr_->command != GearCommand::PARK) {
    convertTwistToWheelsRPM(trans_vel, angular_vel, &left_wheel_rpm, &right_wheel_rpm);
  }

  s1_left_cmd_.command.speed = left_wheel_rpm;
  s1_right_cmd_.command.speed = right_wheel_rpm;

  prev_control_cmd_stamp_ = this->now();
}

void LsdbInterface::onTurnIndicatorsCmd(                                                 // Change to AVA-3510 DIO
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;

  auto hazard_report_msg = HazardLightsReport{};
  hazard_report_msg.stamp = this->now();
  auto turn_indicator_report_msg = TurnIndicatorsReport{};
  turn_indicator_report_msg.stamp = this->now();

  dio_ros_driver::msg::DIOPort dio2_msg, dio3_msg;

  if (!hazard_light_cmd_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "hazard light command is not subscribed");
    return;
  }

  if (hazard_light_cmd_ptr_->command == HazardLightsCommand::ENABLE) {
    hazard_report_msg.report = HazardLightsReport::ENABLE;
    dio2_msg.value = false;
    dio3_msg.value = false;
  } else {
    turn_indicator_report_msg.report = TurnIndicatorsReport::DISABLE;
    switch (msg->command) {
      case TurnIndicatorsCommand::NO_COMMAND:
        turn_indicator_report_msg.report = TurnIndicatorsReport::DISABLE;
        dio2_msg.value = true;
        dio3_msg.value = true;
        break;
      case TurnIndicatorsCommand::DISABLE:
        turn_indicator_report_msg.report = TurnIndicatorsReport::DISABLE;
        dio2_msg.value = true;
        dio3_msg.value = true;
        break;
      case TurnIndicatorsCommand::ENABLE_LEFT:
        turn_indicator_report_msg.report = TurnIndicatorsReport::ENABLE_LEFT;
        dio2_msg.value = false;
        dio3_msg.value = true;
        break;
      case TurnIndicatorsCommand::ENABLE_RIGHT:
        turn_indicator_report_msg.report = TurnIndicatorsReport::ENABLE_RIGHT;
        dio2_msg.value = true;
        dio3_msg.value = false;
        break;
      default:
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *get_clock(), 3000, "error: turn_indicator_cmd = %d", msg->command);
        break;
    }
  }

  turn_indicators_status_pub_->publish(turn_indicator_report_msg);
  hazard_lights_status_pub_->publish(hazard_report_msg);
  dout2_left_blinker_pub_->publish(dio2_msg);
  dout3_right_blinker_pub_->publish(dio3_msg);
}

void LsdbInterface::onHazardLightsCmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_light_cmd_ptr_ = msg;
}

void LsdbInterface::onHeadLightsCmd(
  const autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr msg)
{
  dio_ros_driver::msg::DIOPort dout1_msg;

  switch (msg->command)
  {
  case 1:
    dout1_msg.value = true;
    dout1_front_light_pub_->publish(dout1_msg);
    break;
  case 2:
  case 3:
    dout1_msg.value = false;
    dout1_front_light_pub_->publish(dout1_msg);
    break;
  default:
    break;
  }
}

void LsdbInterface::onGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  using autoware_auto_vehicle_msgs::msg::GearReport;

  gear_cmd_ptr_ = msg;

  auto report_msg = GearReport{};
  report_msg.stamp = msg->stamp;
  if (msg->command == GearCommand::PARK) {
    report_msg.report = GearReport::PARK;
  }
  if (msg->command == GearCommand::REVERSE) {
    report_msg.report = GearReport::REVERSE;
  }
  if (msg->command == GearCommand::DRIVE) {
    report_msg.report = GearReport::DRIVE;
  }
  if (msg->command == GearCommand::LOW) {
    report_msg.report = GearReport::LOW;
  }
  gear_status_pub_->publish(report_msg);
}

void LsdbInterface::onEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void LsdbInterface::convertTwistToWheelsRPM(
  const double trans_vel, const double angular_vel, double * const left_wheel_rpm,
  double * const right_wheel_rpm)
{
  const double rpm_unit = 2 * wheel_radius_ * M_PI / 60.0;

  double left_wheel_vel = (fabs(angular_vel) > 0)
                            ? ((trans_vel / angular_vel) - wheel_tread_ / 2.0) * angular_vel
                            : trans_vel;
  double right_wheel_vel = (fabs(angular_vel) > 0)
                             ? ((trans_vel / angular_vel) + wheel_tread_ / 2.0) * angular_vel
                             : trans_vel;

  // LSDBでは取り付け向きの都合上-1をつける
  left_wheel_vel *= -1.0;

  *left_wheel_rpm = left_wheel_vel / rpm_unit;
  *right_wheel_rpm = right_wheel_vel / rpm_unit;
}

void LsdbInterface::onLsdbLeftStatus(
  const lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr msg)
{
  lsdb_left_status_ptr_ = msg;

  if (!lsdb_right_status_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "right motor status is not subscribed");
    return;
  }

  publishVelocityAndSteering(msg, lsdb_right_status_ptr_);
}

void LsdbInterface::onLsdbRightStatus(
  const lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr msg)
{
  lsdb_right_status_ptr_ = msg;
}

void LsdbInterface::onDin0Estop(const dio_ros_driver::msg::DIOPort::ConstSharedPtr msg)
{
  estop_button_status_ = !msg->value;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LsdbInterface)
