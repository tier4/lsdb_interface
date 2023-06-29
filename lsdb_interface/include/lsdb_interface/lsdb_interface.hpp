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

#ifndef LSDB_INTERFACE__LSDB_INTERFACE_HPP_
#define LSDB_INTERFACE__LSDB_INTERFACE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lsdb_msgs/msg/lsdb_status_stamped.hpp"
#include "lsdb_msgs/msg/lsdb_command_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

class LsdbInterface : public rclcpp::Node
{
public:
  explicit LsdbInterface(const rclcpp::NodeOptions & node_options);

private:
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;
  double speed_scale_factor_;
  double loop_rate_;
  double control_cmd_timeout_sec_;
  bool is_emergency_{false};
  rclcpp::Time prev_control_cmd_stamp_{0, 0, RCL_ROS_TIME};
  bool is_control_command_timeout_;
  bool estop_button_status_{false};

  lsdb_msgs::msg::LsdbCommandStamped s1_right_cmd_, s1_left_cmd_;
  lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr lsdb_right_status_ptr_,
    lsdb_left_status_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_light_cmd_ptr_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  dio_ros_driver::msg::DIOPort dout1_msg_;

  void onAckermannControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onTurnIndicatorsCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onHazardLightsCmd(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void onEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  void onLsdbRightStatus(const lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr msg);
  void onLsdbLeftStatus(const lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr msg);
  void onDin0Estop(const dio_ros_driver::msg::DIOPort::ConstSharedPtr msg);
  void convertTwistToWheelsRPM(
    const double trans_vel, const double angular_vel, double * const left_wheel_rpm,
    double * const right_wheel_rpm);
  void onTimer();
  void publishVehicleControlMode();
  void publishCommand();
  void publishZeroCommand();
  void publishVelocityAndSteering(
    lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr left_msg,
    lsdb_msgs::msg::LsdbStatusStamped::ConstSharedPtr right_msg);

  // Diagnostics
  void setupDiagnosticUpdater();
  void checkDriverErrCode(const int bit_number, diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkInternalErr(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(0, stat);};
  void checkEncoderABZSignalErr(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(1, stat);};
  void checkEncoderUVWSignalErr(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(2, stat);};
  void checkEncoderCountingErr(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(3, stat);};
  void checkDriverTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(4, stat);};
  void checkDriverBusVoltageHigh(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(5, stat);};
  void checkDriverBusVoltageLow(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(6, stat);};
  void checkDriverOutputShortCircuit(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(7, stat);};
  void checkBrakingResistorTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(8, stat);};
  void checkFollowingErrOverRange(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(9, stat);};
  void checkOverload(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(11, stat);};
  void checkSpeedFolowingErrOverRange(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(12, stat);};
  void checkMotorTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(13, stat);};
  void checkSearchingMotorFailed(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(14, stat);};
  void checkCommunicationFailed(diagnostic_updater::DiagnosticStatusWrapper & stat){checkDriverErrCode(15, stat);};
  diagnostic_updater::Updater diagnostic_updater_{this};
  std::map<int, std::string> error_code_map_{
    {0, "Internal error"},
    {1, "Encoder ABZ signal error"},
    {2, "Encoder UVW signal error"},
    {3, "Encoder counting error"},
    {4, "Driver temperature too high"},
    {5, "Driver bus voltage too high"},
    {6, "Driver bus voltage too low"},
    {7, "Driver output short-circuit"},
    {8, "Braking resistor temperature too high"},
    {9, "Following error over-range"},
    {10, "Reserved"},
    {11, "I²T error(Overload)"},
    {12, "Speed following error over-range"},
    {13, "Motor temperature too high"},
    {14, "Searching motor failed(Communication encoder)"},
    {15, "Communication failed"}
  };

  // Subscribe from Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;
  // Subscribe from lsdb
  rclcpp::Subscription<lsdb_msgs::msg::LsdbStatusStamped>::SharedPtr lsdb_left_status_sub_;
  rclcpp::Subscription<lsdb_msgs::msg::LsdbStatusStamped>::SharedPtr lsdb_right_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr right_motor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr left_motor_status_sub_;
  // Subscribe from AVA-3510 Din
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr din0_estop_sub_;

  // Publish to s1
  rclcpp::Publisher<lsdb_msgs::msg::LsdbCommandStamped>::SharedPtr s1_cmd_right_pub_;
  rclcpp::Publisher<lsdb_msgs::msg::LsdbCommandStamped>::SharedPtr s1_cmd_left_pub_;
  // Publish to Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr velocity_kmph_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    steering_wheel_deg_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_charge_status_pub_;
  // Publish to AVA-3510 Dout
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr dout0_brake_light_pub_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr dout1_front_light_pub_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr dout2_right_blinker_pub_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr dout3_left_blinker_pub_;
};

#endif  // LSDB_INTERFACE__LSDB_INTERFACE_HPP_
