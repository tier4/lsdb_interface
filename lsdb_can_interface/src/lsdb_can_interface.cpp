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

#include "lsdb_can_interface/lsdb_can_interface.hpp"
#include "lsdb_can_interface/lsdb_command_list.hpp"

LsdbCanInterface::LsdbCanInterface(const rclcpp::NodeOptions & node_options)
: Node("lsdb_can_interface", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const uint32_t node_id = declare_parameter<int>("node_id", 1);
  sending_can_id_ = (uint32_t)(1536 + node_id);     // 0x600 + node_id
  receiving_can_id_ = (uint32_t)(1408 + node_id);   // 0x580 + node_id
  pdo_fnc_can_id_ = (uint32_t)(384 + node_id);      // 0x180 + node_id

  const double trapezoidal_accel = declare_parameter<double>("trapezoidal_accel", 2.0);
  const double trapezoidal_decel = declare_parameter<double>("trapezoidal_decel", 2.5);
  trapezoidal_accel_ = (uint32_t)(trapezoidal_accel * 256.0 * 4096.0 / 15625.0);
  trapezoidal_decel_ = (uint32_t)(trapezoidal_decel * 256.0 * 4096.0 / 15625.0);

  const double status_loop_rate_hz = declare_parameter<double>("status_loop_rate_hz", 10.0);

  // Subscriber
  command_sub_ = this->create_subscription<lsdb_msgs::msg::LsdbCommandStamped>(
    "~/input/command", 1, std::bind(&LsdbCanInterface::onLsdbCommand, this, _1));
  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", 10, std::bind(&LsdbCanInterface::onCanMsg, this, _1));

  // Publisher
  status_pub_ = this->create_publisher<lsdb_msgs::msg::LsdbStatusStamped>("~/output/status", 10);
  can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);

  // Timer
  const auto period_ns = rclcpp::Rate(status_loop_rate_hz).period();
  on_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&LsdbCanInterface::onTimer, this));
}

void LsdbCanInterface::onTimer()
{
  using namespace lsdb_command_list;

  // Initial settings to LSDB driver
  if (initialize_status_ == InitStatus::Start) {
    sendCommand(CommandID::eTrapezoidal_acceleration, trapezoidal_accel_, ComType::write);
    return;
  }

  // send READ status commands
  sendCommand(CommandID::eControl_word, (int16_t)0x00, ComType::read);
  sendCommand(CommandID::eDrive_error_status_word_1, (uint16_t)0x00, ComType::read);
  sendCommand(CommandID::eEmergency_stop_order, (uint8_t)0x00, ComType::read);
}

void LsdbCanInterface::lsdbInitialization(const uint32_t can_cmd_id, const bool is_write_success)
{
  using namespace lsdb_command_list;

  switch (initialize_status_)
  {
  case InitStatus::Start:
    // TODO
    // break;
  case InitStatus::Setting_trapezoidal_accel:
      if (can_cmd_id == spec_map[CommandID::eTrapezoidal_acceleration].can_cmd_id && is_write_success) {
        sendCommand(CommandID::eTrapezoidal_deceleration, trapezoidal_decel_, ComType::write);
        initialize_status_ = InitStatus::Setting_trapezoidal_decel;
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] Write commnad failed: trapezoidal_accel");
      }
      break;
  case InitStatus::Setting_trapezoidal_decel:
      if (can_cmd_id == spec_map[CommandID::eTrapezoidal_deceleration].can_cmd_id && is_write_success) {
        sendCommand(CommandID::eAction_mode, (int8_t)0x03, ComType::write);
        initialize_status_ = InitStatus::Setting_action_mode;
      } else {
        sendCommand(CommandID::eTrapezoidal_deceleration, trapezoidal_decel_, ComType::write);
        RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] Write commnad failed: trapezoidal_decel");
      }
      break;
  case InitStatus::Setting_action_mode:
      if (can_cmd_id == spec_map[CommandID::eAction_mode].can_cmd_id && is_write_success) {
        sendCommand(CommandID::eControl_word, (int16_t)0x0F, ComType::write);
        initialize_status_ = InitStatus::Setting_motor_operation;
      } else {
        sendCommand(CommandID::eAction_mode, (int8_t)0x03, ComType::write);
        RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] Write commnad failed: action_mode");
      }
      break;
  case InitStatus::Setting_motor_operation:
      if (can_cmd_id == spec_map[CommandID::eControl_word].can_cmd_id && is_write_success) {
        sendCommand(CommandID::eSimple_PDO_function, (int16_t)0x01, ComType::write);
        initialize_status_ = InitStatus::Setting_pdo_function;
      } else {
        sendCommand(CommandID::eControl_word, (int16_t)0x0F, ComType::write);
        RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] Write commnad failed: motor_operation");
      }
      break;
  case InitStatus::Setting_pdo_function:
      if (can_cmd_id == spec_map[CommandID::eSimple_PDO_function].can_cmd_id && is_write_success) {
        initialize_status_ = InitStatus::End;
        RCLCPP_INFO_STREAM(this->get_logger(), "[lsdbInitialization] Initialization finished.");
      } else {
        sendCommand(CommandID::eSimple_PDO_function, (int16_t)0x01, ComType::write);
        RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] Write commnad failed: pdo_function");
      }
      break;
  case InitStatus::End:
      RCLCPP_INFO_STREAM(this->get_logger(), "[lsdbInitialization] Initialization finished.");
      break;
  default:
      RCLCPP_ERROR_STREAM(this->get_logger(), "[lsdbInitialization] InitStatus Error.");
      break;
  }

}

void LsdbCanInterface::onLsdbCommand(const lsdb_msgs::msg::LsdbCommandStamped::ConstSharedPtr msg)
{
  using namespace lsdb_command_list;
  sendCommand(CommandID::eTarget_speed, transRpmToTargetVelDec(msg->command.speed), ComType::write);
}

template <typename T>
void LsdbCanInterface::sendCommand(const lsdb_command_list::CommandID & CommandID, const T val, const lsdb_command_list::ComType & com_type)
{
  using namespace lsdb_command_list;

  can_msgs::msg::Frame send_can_msg;
  send_can_msg.header.stamp = this->now();
  send_can_msg.id = sending_can_id_;
  send_can_msg.dlc = 8;

  Spec data_spec;
  try {
    data_spec = spec_map[CommandID];
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[sendCommand]Spec error");
    return;
  }

  // set CMD
  int32_t write_size = stoi(data_spec.write_size.substr(0, 2));
  if (com_type == ComType::read) {
    send_can_msg.data.at(0) = SendingCmd_ReadData;
  } else if (com_type == ComType::write) {
    if (write_size == 32) {
      send_can_msg.data.at(0) = SendingCmd_Write4Bytes;
    } else if (write_size == 16) {
      send_can_msg.data.at(0) = SsndingCmd_Write2Bytes;
    } else if (write_size <= 8) {
      send_can_msg.data.at(0) = SendingCmd_Write1Byte;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[sendCommand]read and write size error");
      return;
    }
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[sendCommand]type error");
    return;
  }

  // set Index
  send_can_msg.data.at(1) = (uint8_t)(data_spec.can_cmd_id >> 8) & 0xFF;
  send_can_msg.data.at(2) = (uint8_t)(data_spec.can_cmd_id >> 16) & 0xFF;
  send_can_msg.data.at(3) = (uint8_t)(data_spec.can_cmd_id) & 0xFF;

  // set Data
  std::size_t size = write_size / 8;
  if (size < 1) {
    size = 0;
    RCLCPP_INFO_STREAM(this->get_logger(), "create send data size 0");
  }
  for (std::size_t byte = 0; byte < size; byte++) {
    uint8_t byte_data = (uint8_t)(val >> (byte * 8)) & 0xFF;
    send_can_msg.data.at(4 + byte) = byte_data;
  }

  can_pub_->publish(send_can_msg);
}

void LsdbCanInterface::onCanMsg(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  using namespace lsdb_command_list;

  // PDO function process
  if (msg->id == pdo_fnc_can_id_) {
    int32_t motor_speed_rpm_raw = 0;  // [1000/rpm]
    for (std::size_t byte = 0; byte < 4; byte++) {
      motor_speed_rpm_raw |= (int32_t)(msg->data.at(byte)) << (byte * 8);
    }
    if (abs(motor_speed_rpm_raw) < 100) {
      motor_speed_rpm_raw = 0;
    }
    lsdb_status_.status.motor_speed_rpm = static_cast<double>(motor_speed_rpm_raw) * 0.001;

    lsdb_status_.stamp = this->now();
    status_pub_->publish(lsdb_status_);

    return;
  } else if (msg->id != receiving_can_id_) {
    return;
  }

  const uint32_t can_cmd_id = getCmdIndex(msg->data);
  bool isWriteSuccess = false;
  std::size_t read_size_byte = 0;

  // get CMD status
  switch (msg->data.at(0))
  {
  case ReceivingCmd_Read1Byte:
    read_size_byte = 1;
  case ReceivingCmd_Read2Bytes:
    read_size_byte = 2;
  case ReceivingCmd_Read4Bytes:
    read_size_byte = 4;
  case ReceivingCmd_WriteSuccess:
    isWriteSuccess = true;
    break;
  case ReceivingCmd_WriteFaild:
    isWriteSuccess = false;
    RCLCPP_ERROR_STREAM(this->get_logger(), "CAN message writing data failed.");
    break;
  default:
    RCLCPP_ERROR_STREAM(this->get_logger(), "CAN message CMD data error.");
    break;
  }

  // Initialization
  if (initialize_status_ != InitStatus::End) {
    lsdbInitialization(can_cmd_id, isWriteSuccess);
  }

  // status command processing
  if (can_cmd_id == spec_map[CommandID::eEmergency_stop_order].can_cmd_id) {
    uint8_t tmp = 0;
    transLsdbData(read_size_byte, msg->data, &tmp);
    lsdb_status_.status.emergency_stop  = static_cast<bool>(tmp);
  } else if (can_cmd_id == spec_map[CommandID::eControl_word].can_cmd_id) {
    transLsdbData(read_size_byte, msg->data, &lsdb_status_.status.control_word.data);
  } else if (can_cmd_id == spec_map[CommandID::eDrive_error_status_word_1].can_cmd_id) {
    uint16_t tmp = 0;
    uint16_t err_bit = 1;
    transLsdbData(read_size_byte, msg->data, &tmp);
    for (size_t i = 0; i < lsdb_status_.status.driver_err_code.data.size(); i++) {
      if ((uint16_t)tmp & err_bit) {
        lsdb_status_.status.driver_err_code.data.at(i) = 1;
      }
      err_bit <<= 1;
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LsdbCanInterface)
