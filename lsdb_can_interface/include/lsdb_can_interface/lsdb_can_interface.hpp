// Copyright 2023 Tier IV, Inc.
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

#ifndef LSDB_CAN_INTERFACE__LSDB_CAN_INTERFACE_HPP_
#define LSDB_CAN_INTERFACE__LSDB_CAN_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "lsdb_can_interface/lsdb_command_list.hpp"

#include "can_msgs/msg/frame.hpp"

#include "lsdb_msgs/msg/driver_err_code.hpp"
#include "lsdb_msgs/msg/lsdb_debug_stamped.hpp"
#include "lsdb_msgs/msg/lsdb_status_stamped.hpp"
#include "lsdb_msgs/msg/lsdb_command_stamped.hpp"

class LsdbCanInterface : public rclcpp::Node
{
public:
  LsdbCanInterface(const rclcpp::NodeOptions & node_options);
  // ~LsdbCanInterface();

private:

  enum class InitStatus : int32_t {
    Start,                      // 初期状態
    Setting_trapezoidal_accel,  // 台形加速度設定中
    Setting_trapezoidal_decel,  // 台形減速度設定中
    Setting_action_mode,        // 動作モード設定中
    Setting_motor_operation,    // モーター動作設定中
    Setting_pdo_function,       // PDO function設定中
    End,                        // 終了
  };
  InitStatus initialize_status_{InitStatus::Start};

  // Parameter
  uint32_t sending_can_id_;
  uint32_t receiving_can_id_;
  uint32_t pdo_fnc_can_id_;
  uint32_t trapezoidal_accel_;  // accel
  uint32_t trapezoidal_decel_;  // decel

  // Variable
  lsdb_msgs::msg::LsdbStatusStamped lsdb_status_;

  // Funcsion
  void startInitialization();
  void lsdbInitialization(const uint32_t can_cmd_id, const bool is_write_success);
  void onTimer();
  template <typename T>
  void sendCommand(const lsdb_command_list::CommandID & CommandID, const T val, const lsdb_command_list::ComType & com_type);

  // Transformation
  template <typename T>
  void transLsdbData(const std::size_t size_byte, const std::array<unsigned char, 8> & data, T * transdata) {
    for (std::size_t byte = 0; byte < size_byte; byte++) {
      *transdata |= (T)(data.at(4 + byte)) << (byte * 8);
    }
  }
  uint32_t transRpmToTargetVelDec(const double rpm) { return (uint32_t)(rpm * 512.0 * 4096.0 / 1875.0); }
  uint32_t getCmdIndex(const std::array<unsigned char, 8> & data) { // get CAN command Index + Subindex
    return (((uint32_t)data.at(2) << 16) | ((uint32_t)data.at(1) << 8) | (uint32_t)data.at(3));
  }

  // Timer
  rclcpp::TimerBase::SharedPtr on_timer_;

  // Callback
  void onLsdbCommand(const lsdb_msgs::msg::LsdbCommandStamped::ConstSharedPtr msg);
  void onCanMsg(const can_msgs::msg::Frame::ConstSharedPtr msg);

  // Subscriber
  rclcpp::Subscription<lsdb_msgs::msg::LsdbCommandStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  // Publisher
  rclcpp::Publisher<lsdb_msgs::msg::LsdbStatusStamped>::SharedPtr status_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

};

#endif  // LSDB_CAN_INTERFACE__LSDB_CAN_INTERFACE_HPP_
