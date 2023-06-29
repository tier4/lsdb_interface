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

#include "lsdb_interface/lsdb_interface.hpp"

#include <string>

void LsdbInterface::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("LSDB Motor Driver");
  diagnostic_updater_.add("Internal error", this, &LsdbInterface::checkInternalErr);
  diagnostic_updater_.add("Encoder ABZ signal error", this, &LsdbInterface::checkEncoderABZSignalErr);
  diagnostic_updater_.add("Encoder UVW signal error", this, &LsdbInterface::checkEncoderUVWSignalErr);
  diagnostic_updater_.add("Encoder counting error", this, &LsdbInterface::checkEncoderCountingErr);
  diagnostic_updater_.add("Driver temperature too high", this, &LsdbInterface::checkDriverTempHigh);
  diagnostic_updater_.add("Driver bus voltage too high", this, &LsdbInterface::checkDriverBusVoltageHigh);
  diagnostic_updater_.add("Driver bus voltage too low", this, &LsdbInterface::checkDriverBusVoltageLow);
  diagnostic_updater_.add("Driver output short-circuit", this, &LsdbInterface::checkDriverOutputShortCircuit);
  diagnostic_updater_.add("Braking resistor temperature too high", this, &LsdbInterface::checkBrakingResistorTempHigh);
  diagnostic_updater_.add("Following error over-range", this, &LsdbInterface::checkFollowingErrOverRange);
  diagnostic_updater_.add("I²T error(Overload)", this, &LsdbInterface::checkOverload);
  diagnostic_updater_.add("Speed following error over-range", this, &LsdbInterface::checkSpeedFolowingErrOverRange);
  diagnostic_updater_.add("Motor temperature too high", this, &LsdbInterface::checkMotorTempHigh);
  diagnostic_updater_.add("Searching motor failed(Communication encoder)", this, &LsdbInterface::checkSearchingMotorFailed);
  diagnostic_updater_.add("Communication failed", this, &LsdbInterface::checkCommunicationFailed);
}

void LsdbInterface::checkDriverErrCode(const int bit_number, diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string summary_msg = "OK";
  bool get_driver_status = true;

  if (!lsdb_left_status_ptr_) {
      RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Left LSDB status msg not received");
      summary_msg = "Left LSDB status msg not received";
      get_driver_status = false;
  }
  if (!lsdb_right_status_ptr_) {
      RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Right LSDB status msg not received");
      if (!get_driver_status) {
          summary_msg = "Both LSDB status msg not received";
      }
      else {
          summary_msg = "Right LSDB status msg not received";
          get_driver_status = false;
      }
  }
  if (!get_driver_status) {
      stat.summary(DiagnosticStatus::ERROR, summary_msg);
      return;
  }

  std::string additional_msg = "";
  if (lsdb_left_status_ptr_->status.driver_err_code.data.at(bit_number) == 1) {
    level = DiagnosticStatus::ERROR;
    additional_msg = "Left error";
  }
  if (lsdb_right_status_ptr_->status.driver_err_code.data.at(bit_number) == 1) {
    if (level == DiagnosticStatus::ERROR) {
      additional_msg = "Both error";
    }
    else {
      level = DiagnosticStatus::ERROR;
      additional_msg = "Right error";
    }
  }

  if (level == DiagnosticStatus::ERROR) {
    summary_msg = error_code_map_.at(bit_number) + ": " + additional_msg;
  }
  stat.summary(level, summary_msg);

}

void LsdbInterface::checkInternalErr(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(0, stat);
}

void LsdbInterface::checkEncoderABZSignalErr(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(1, stat);
}

void LsdbInterface::checkEncoderUVWSignalErr(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(2, stat);
}

void LsdbInterface::checkEncoderCountingErr(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(3, stat);
}

void LsdbInterface::checkDriverTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(4, stat);
}

void LsdbInterface::checkDriverBusVoltageHigh(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(5, stat);
}

void LsdbInterface::checkDriverBusVoltageLow(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(6, stat);
}

void LsdbInterface::checkDriverOutputShortCircuit(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(7, stat);
}

void LsdbInterface::checkBrakingResistorTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(8, stat);
}

void LsdbInterface::checkFollowingErrOverRange(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(9, stat);
}

void LsdbInterface::checkOverload(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(11, stat);
}

void LsdbInterface::checkSpeedFolowingErrOverRange(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(12, stat);
}

void LsdbInterface::checkMotorTempHigh(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(13, stat);
}

void LsdbInterface::checkSearchingMotorFailed(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(14, stat);
}

void LsdbInterface::checkCommunicationFailed(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  checkDriverErrCode(15, stat);
}
