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

#include "lsdb_serial_interface/lsdb_serial_interface.hpp"

#include <boost/exception/all.hpp>

#include <regex>
#include <sstream>
#include <streambuf>
#include <thread>

LsdbSerialInterface::LsdbSerialInterface(const rclcpp::NodeOptions & node_options)
: Node("lsdb_serial_interface", node_options), serial_(io_)
{
  using std::placeholders::_1;

  // 1. launchからのパラメータを取得
  const double trapezoidal_accel = declare_parameter<double>("trapezoidal_accel", 2.0);
  const double trapezoidal_decel = declare_parameter<double>("trapezoidal_decel", 2.5);
  trapezoidal_accel_ = (uint32_t)(trapezoidal_accel * 256.0 * 4096.0 / 15625.0);
  trapezoidal_decel_ = (uint32_t)(trapezoidal_decel * 256.0 * 4096.0 / 15625.0);
  const auto loop_rate_hz = declare_parameter<double>("loop_rate_hz", 50.0);

  // Subscriber
  command_sub_ = this->create_subscription<lsdb_msgs::msg::LsdbCommandStamped>(
    "~/input/command", 10, std::bind(&LsdbSerialInterface::onLsdbCommand, this, _1));

  // Publisher
  status_pub_ = this->create_publisher<lsdb_msgs::msg::LsdbStatusStamped>("~/output/status", 10);

  // Debug status publisher
  debug_status_pub_ =
    this->create_publisher<lsdb_msgs::msg::LsdbDebugStamped>("~/output/debug/status", 10);

  // serial
  devname_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");

  openSerial();

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_hz).period();
  on_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&LsdbSerialInterface::onTimer, this));

  // Thread for read serial
  std::thread read_thr(&LsdbSerialInterface::readSerial, this);
  read_thr.detach();
}

LsdbSerialInterface::~LsdbSerialInterface()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "in destructor");
  closeSerial();
  RCLCPP_INFO_STREAM(this->get_logger(), "serial is closed");
}

std::vector<uint8_t> LsdbSerialInterface::receive()
{
  using boost::asio::buffers_begin;
  using boost::asio::buffers_end;
  using boost::asio::read;
  using boost::asio::streambuf;
  using boost::asio::transfer_exactly;

  std::vector<uint8_t> buffer_vector;
  buffer_vector.reserve(SIZE);

  // 受信データのバリデーションとエラーハンドリング
  streambuf response;
  read(serial_, response, transfer_exactly(SIZE));

  // FIRST BYTEの位置を探す
  const auto itr =
    std::find(buffers_begin(response.data()), buffers_end(response.data()), FIRST_BYTE);

  // FIRST BYTEが先頭じゃない場合
  if (itr != buffers_begin(response.data())) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "first byte invalid");

    // FIRST BYTEが先頭になるように削除
    const auto distance = std::distance(buffers_begin(response.data()), itr);
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "FIRST_BYTE is: " << distance << "th byte, delete front");
    response.consume(distance);

    // SIZEのバイトになるようにデータを追加
    read(serial_, response, transfer_exactly(SIZE - response.size()));
  }
  for (auto it = buffers_begin(response.data()); it != buffers_end(response.data()); ++it) {
    buffer_vector.push_back(static_cast<uint8_t>(*it));
  }

  return buffer_vector;
}

void LsdbSerialInterface::onTimer()
{
  // 現在速度要求を定周期で送信
  send(CommandID::eActual_speed_0001rpm, (int32_t)0x00, SerialComType::read);
  send(CommandID::eControl_word, (int16_t)0x00, SerialComType::read);
  send(CommandID::eDrive_error_status_word_1, (uint16_t)0x00, SerialComType::read);
  send(CommandID::eEmergency_stop_order, (uint8_t)0x00, SerialComType::read);

  publishLsdbStatus();
}

void LsdbSerialInterface::openSerial()
{
  using boost::asio::serial_port_base;

  while (!serial_.is_open()) {
    serial_.open(devname_);
    serial_.set_option(serial_port_base::baud_rate(115200));
    serial_.set_option(serial_port_base::character_size(8));
    serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    RCLCPP_INFO_STREAM(this->get_logger(), "open serial:" << devname_);
    initialSequence();
  }
}

void LsdbSerialInterface::readSerial()
{
  while (rclcpp::ok()) {
    const auto buffer = receive();
    receiveData(buffer);
  }
}

void LsdbSerialInterface::publishLsdbStatus()
{
  // receiveData()、initialSequence()、updateStatus()で更新される値があるためロック
  std::lock_guard<std::mutex> lock(mutex_);
  lsdb_msgs::msg::LsdbStatusStamped status;
  status.stamp = this->now();
  status.status.motor_speed_rpm = rpm_;
  status.status.init_status = static_cast<int>(initialize_status_);
  status.status.emergency_stop = emergency_stop_;
  status.status.control_word = current_control_word_;
  status.status.driver_err_code = driver_err_code_;
  status_pub_->publish(status);

  RCLCPP_DEBUG(this->get_logger(), "Status is published, reset");
}

void LsdbSerialInterface::onLsdbCommand(
  const lsdb_msgs::msg::LsdbCommandStamped::ConstSharedPtr msg)
{
  send(CommandID::eTarget_speed, transRpmToLsb(msg->command.speed), SerialComType::write);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー初期化シーケンス
/// @fn             initialSequence
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
void LsdbSerialInterface::initialSequence()
{
  // publishLsdbStatusで値が参照されるためロック
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialize_status_ == InitStatus::Start) {
    RCLCPP_INFO(this->get_logger(), "init status: start");
    // 台形加速度を設定
    send(CommandID::eTrapezoidal_acceleration, trapezoidal_accel_, SerialComType::write);
    initialize_status_ = InitStatus::Setting_trapezoidal_accel;
  }

  while (initialize_status_ != InitStatus::End) {
    const auto buffer = receive();
    int16_t id = getSerialId(buffer);

    if (id == (int16_t)spec_map[CommandID::eTrapezoidal_acceleration].serial_id) {
      RCLCPP_INFO(this->get_logger(), "init status: trapezoidal accel");
      // 台形加速度を設定
      if (
        initialize_status_ == InitStatus::Setting_trapezoidal_accel &&
        send(CommandID::eTrapezoidal_deceleration, trapezoidal_decel_, SerialComType::write)) {
        initialize_status_ = InitStatus::Setting_trapezoidal_decel;
      }
    } else if (id == (int16_t)spec_map[CommandID::eTrapezoidal_deceleration].serial_id) {
      RCLCPP_INFO(this->get_logger(), "init status: trapezoidal decel");
      // 台形減速度設定応答
      if (
        initialize_status_ == InitStatus::Setting_trapezoidal_decel &&
        send(CommandID::eAction_mode, (int8_t)0x03, SerialComType::write)) {
        initialize_status_ = InitStatus::Setting_action_mode;
      }
    } else if (id == (int16_t)spec_map[CommandID::eAction_mode].serial_id) {
      RCLCPP_INFO(this->get_logger(), "init status: action mode");
      // 動作モード設定応答
      if (
        initialize_status_ == InitStatus::Setting_action_mode &&
        send(CommandID::eControl_word, (int16_t)0x0F, SerialComType::write)) {
        initialize_status_ = InitStatus::Setting_motor_operation;
      }
    } else if (id == (int16_t)spec_map[CommandID::eControl_word].serial_id) {  // 制御設定応答
      RCLCPP_INFO(this->get_logger(), "init status: eEnd");
      initialize_status_ = InitStatus::End;
    }
  }  // while
}

template <typename T>
bool LsdbSerialInterface::send(
  const CommandID & CommandID, const T val, const SerialComType & com_type)
{
  std::vector<uint8_t> data(SIZE, 0x00);

  // 送信データを生成して送信
  if (true == createSendData(CommandID, val, com_type, &data)) {
    publishDebug(data, SerialComType::write);
    write(data);
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー 送信データの生成
/// @fn             createSendData
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
template <typename T>
bool LsdbSerialInterface::createSendData(
  const CommandID & CommandID, const T val, const SerialComType & com_type,
  std::vector<uint8_t> * send_data)
{
  // 指定されたコマンドのSPECを取得
  Spec data_spec;
  try {
    data_spec = spec_map[CommandID];
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[createSendData]Spec error");
    return false;
  }

  // IDの設定
  send_data->at(0) = FIRST_BYTE;

  //-------------//
  // CMD部の設定 //
  //-------------//
  int32_t write_size = stoi(data_spec.write_size.substr(0, 2));
  if (com_type == SerialComType::read) {
    if (write_size == 32) {
      send_data->at(1) = _cReadCmd_Host_32bitLSB;
    } else if (write_size == 16) {
      send_data->at(1) = _cReadCmd_Host_16bitLSB;
    } else if (write_size <= 8) {
      send_data->at(1) = _cReadCmd_Host_8bitLSB;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[createSendData]read only size error");
      return false;
    }
  } else if (com_type == SerialComType::write) {
    if (write_size == 32) {
      send_data->at(1) = _cSendCmd_Host_32bitLSB;
    } else if (write_size == 16) {
      send_data->at(1) = _cSendCmd_Host_16bitLSB;
    } else if (write_size <= 8) {
      send_data->at(1) = _cSendCmd_Host_8bitLSB;
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[createSendData]read and write size error");
      return false;
    }
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "[createSendData]type error");
    return false;
  }

  //----------------------//
  // Object Addressの設定 //
  //----------------------//
  send_data->at(2) = (uint8_t)(data_spec.serial_id >> 8) & 0xFF;
  send_data->at(3) = (uint8_t)(data_spec.serial_id) & 0xFF;

  //-------------//
  // Err部の設定 //
  //-------------//
  send_data->at(4) = 0x00;

  //--------------//
  // Data部の設定 //
  //--------------//
  std::size_t size = write_size / 8;
  if (size < 1) {
    size = 0;
    RCLCPP_INFO_STREAM(this->get_logger(), "create send data size 0");
  }
  for (std::size_t byte = 0; byte < size; byte++) {
    uint8_t byte_data = (uint8_t)(val >> (byte * 8)) & 0xFF;
    send_data->at(8 - byte) = byte_data;
  }

  //----------------//
  // check sumの算出 //
  //----------------//
  send_data->at(9) = chksum(*send_data);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー チェックサム
/// @fn             chksum
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
uint8_t LsdbSerialInterface::chksum(const std::vector<uint8_t> & data)
{
  uint8_t chk = 0;

  for (std::size_t i = 0; i < data.size() - 1; i++) {
    chk += data.at(i);
  }

  return chk;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー 送信データの書き込み
/// @fn             write
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
void LsdbSerialInterface::write(const std::vector<uint8_t> & send_data)
{
  boost::asio::write(serial_, boost::asio::buffer(send_data, SIZE));
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバーコマンド受信コールバック
/// @fn             receiveData
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
bool LsdbSerialInterface::receiveData(const std::vector<uint8_t> & buffer)
{
  // publishLsdbStatusで値が参照されるためロック
  std::lock_guard<std::mutex> lock(mutex_);
  int16_t id = getSerialId(buffer);

  // 受信データのエラーチェック
  std::optional<SerialErrs> check_result = receiveErrChk(buffer);
  if (check_result.has_value()) {
    publishDebug(buffer, SerialComType::read, check_result.value());
    RCLCPP_ERROR(this->get_logger(), "receiveErrChk Error");
    return false;
  }
  publishDebug(buffer, SerialComType::read);

  int32_t tmp = 0;
  // 現在の実速度（0.001単位）を取得
  if (id == (int16_t)spec_map[CommandID::eActual_speed_0001rpm].serial_id) {
    // LSDBからの値を変換
    transLsdbData(spec_map[CommandID::eActual_speed_0001rpm], buffer, &tmp);
    if (abs(tmp) < 100) {
      tmp = 0;
    }
    // 現在の実速度を保持
    rpm_ = static_cast<double>(tmp) * 0.001;
  } else if (id == (int16_t)spec_map[CommandID::eEmergency_stop_order].serial_id) {
    transLsdbData(spec_map[CommandID::eEmergency_stop_order], buffer, &tmp);
    emergency_stop_ = static_cast<bool>(tmp);
  } else if (id == (int16_t)spec_map[CommandID::eControl_word].serial_id) {
    transLsdbData(spec_map[CommandID::eControl_word], buffer, &tmp);
    if ((uint8_t)tmp == 0x06) {
      current_control_word_.data = lsdb_msgs::msg::ControlWord::DISABLE;  // disable
    } else if ((uint8_t)tmp == 0xf0) {
      current_control_word_.data = lsdb_msgs::msg::ControlWord::ENABLE;  // enable
    } else if ((uint8_t)tmp == 0x86) {
      current_control_word_.data = lsdb_msgs::msg::ControlWord::CLEAR_ERR;  // clear error
    }
  } else if (id == (int16_t)spec_map[CommandID::eDrive_error_status_word_1].serial_id) {
    transLsdbData(spec_map[CommandID::eDrive_error_status_word_1], buffer, &tmp);
    uint16_t err_bit = 1;
    for (size_t i = 0; i < driver_err_code_.data.size(); i++) {
      if ((uint16_t)tmp & err_bit) {
        driver_err_code_.data.at(i) = 1;
      }
      err_bit <<= 1;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー エラー判定
/// @fn             receiveErrChk
/// @author         Atsushi.Kondo
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
std::optional<LsdbSerialInterface::SerialErrs> LsdbSerialInterface::receiveErrChk(
  const std::vector<uint8_t> & rcv_data)
{
  SerialErrs ret_errs{false, "", ""};

  //-------------------//
  // チェックサムチェック //
  //-------------------//
  if (rcv_data.at(9) != chksum(rcv_data)) {
    ret_errs.checksum_error = true;
    RCLCPP_ERROR_STREAM(this->get_logger(), "[receiveErrChk]Check Sum Error");
    return ret_errs;
  }
  //-----------------------//
  // CMD部のエラーチェック //
  //-----------------------//
  const auto cmd = rcv_data.at(1);
  if ((cmd == _cReadCmd_Back_AdrErr) || (cmd == _cSendCmd_Back_AdrErr)) {
    ret_errs.cmd_err = "[CMD error] The object address does not exist";
    RCLCPP_ERROR_STREAM(this->get_logger(), ret_errs.cmd_err);
  } else if (cmd == _cReadCmd_Back_Invalid) {
    ret_errs.cmd_err = "[CMD error] Invalid data area";
    RCLCPP_ERROR_STREAM(this->get_logger(), ret_errs.cmd_err);
  } else if (cmd == _cSendCmd_Back_TypeErr) {
    ret_errs.cmd_err = "[CMD error] Data type does not match";
    RCLCPP_ERROR_STREAM(this->get_logger(), ret_errs.cmd_err);
  } else if (cmd == _cSendCmd_Back_WriteErr) {
    ret_errs.cmd_err = "[CMD error] Object address is not writable";
    RCLCPP_ERROR_STREAM(this->get_logger(), ret_errs.cmd_err);
  } else {
    // エラーなし
  }

  //------------------------//
  // ErrR部のエラーチェック //
  //------------------------//
  const auto err = rcv_data.at(4);
  if (err != 0x00) {
    if (err == _cError_Reserved) {
      ret_errs.err_code = "[ErrR error] Reserved";
    } else if (err == _cError_Unacceptable) {
      ret_errs.err_code = "[ErrR error] Tolerance exceeded";
    } else if (err == _cError_Encoder) {
      ret_errs.err_code = "[ErrR error] Encoder error";
    } else if (err == _cError_Motor_Overload) {
      ret_errs.err_code = "[ErrR error] Motor overload";
    } else if (err == _cError_Drive_Temp_High) {
      ret_errs.err_code = "[ErrR error] Driver temperature is high";
    } else if (err == _cError_DC_Voltage_High) {
      ret_errs.err_code = "[ErrR error] DC bus voltage is too high";
    } else if (err == _cError_DC_Voltage_Low) {
      ret_errs.err_code = "[ErrR error] DC bus voltage is too low";
    } else if (err == _cError_Short_Circuit) {
      ret_errs.err_code = "[ErrR error] Driver output short circuit";
    } else {
      // エラーなし
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), ret_errs.err_code);
  }

  if (!ret_errs.checksum_error && ret_errs.cmd_err == "" && ret_errs.err_code == "") {
    return std::nullopt;
  } else {
    return ret_errs;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル通信ドライバー LSDBデータ変換
/// @fn             transLsdbData
/// @author         Atsushi.Kondo
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
template <typename T>
void LsdbSerialInterface::transLsdbData(
  const Spec & data_spec, const std::vector<uint8_t> & rcv_data, T * transdata)
{
  std::size_t read_size = stoi(data_spec.read_size.substr(0, 2));
  std::size_t size = read_size / 8;
  if (size < 1) {
    size = 0;
  }
  for (std::size_t byte = 0; byte < size; byte++) {
    *transdata |= (T)(rcv_data.at(8 - byte)) << (byte * 8);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief          シリアル制御(linux)クラスデストラクタ
/// @date           XXXX/XX/XX
///
////////////////////////////////////////////////////////////////////////////////
void LsdbSerialInterface::closeSerial()
{
  if (serial_.is_open()) {
    // モーター電源のOFFを送信
    send(CommandID::eControl_word, (int16_t)0x06, SerialComType::write);
    serial_.close();
    RCLCPP_INFO_STREAM(this->get_logger(), "close serial");
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "already closed");
  }
}

void LsdbSerialInterface::publishDebug(
  const std::vector<uint8_t> & buffer, const SerialComType & com_type,
  const SerialErrs & serial_errs)
{
  lsdb_msgs::msg::LsdbDebugStamped debug_status;
  debug_status.stamp = this->now();
  debug_status.status.checksum_err = serial_errs.checksum_error;
  debug_status.status.cmd_err = serial_errs.cmd_err;
  debug_status.status.err_code = serial_errs.err_code;
  for (const auto & e : buffer) {
    std::ostringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(e);
    debug_status.status.serial_str += "0x" + ss.str() + " ";
    if (com_type == SerialComType::read)
      debug_status.status.com_type = "read";
    else if (com_type == SerialComType::write)
      debug_status.status.com_type = "write";
  }
  debug_status_pub_->publish(debug_status);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LsdbSerialInterface)
