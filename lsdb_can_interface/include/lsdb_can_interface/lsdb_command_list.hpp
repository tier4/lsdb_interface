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

#ifndef LSDB_CAN_INTERFACE__LSDB_COMMAND_LIST_HPP_
#define LSDB_CAN_INTERFACE__LSDB_COMMAND_LIST_HPP_

#include "rclcpp/rclcpp.hpp"

namespace lsdb_command_list
{

enum class CommandID : int32_t {
  eCurrent_motor_model_code = 0,     // 現在のモーターモデルコード
  eMotor_model_code,                 // モーターモデルコード
  eFeedback_type,                    // フィードバックタイプ
  eEncoder_resolution,               // エンコーダの分解能
  eNumber_of_motor_pole_pairs,       // モーターポールペアの数
  eExcitation_mode,                  // 励起モード
  eExcitation_current,               // 励起電流
  eExcitation_time,                  // 励起時間
  eMotor_overload_current,           // モーター過負荷電流
  eMotor_overload_time_constant,     // モーター過負荷時定数
  eMaximum_motor_current,            // 最大モーター電流
  eMotor_rotation_direction,         // モーターの回転方向
  eMotor_rated_speed,                // モーター定格速度
  eMotor_rated_power,                // モーター定格電力
  eMotor_hole_angle,                 // モーターホール角
  eAction_mode,                      // 動作モード
  eControl_word,                     // 制御語
  eEffective_Action_mode,            // 効果的な作業モード
  eDrive_status_word,                // ドライブステータスワード
  eDrive_error_status_word_1,        // ドライブエラーステータスワード1
  eEmergency_stop_order,             // 非常停止命令
  eAbsolute_target_position,         // 絶対目標位置
  eRelative_target_position,         // 相対目標位置
  eTrapezoidal_speed,                // 台形の速度
  eTrapezoidal_speed_rpm,            // 台形速度rpm
  eTrapezoidal_acceleration,         // 台形加速度
  eTrapezoidal_deceleration,         // 台形減速
  eSudden_stop_deceleration,         // 急停止減速
  eTarget_speed_rpm,                 // 目標速度rpm
  eTarget_speed,                     // 目標速度
  eTarget_current,                   // 目標電流
  eTarget_current_limit,             // 目標電流制限
  eMaximum_speed_limit_rpm,          // 最高速度制限rpm
  ePosition_error_window,            // エラーウィンドウに続く位置
  eWork_mode_3,                      // 作業モード3
  ePosition_loop_speed_feedforward,  // 位置ループ速度フィードフォワード
  ePosition_loop_proportional_gain,  // 位置ループ比例ゲイン
  eSpeed_loop_proportional_gain,     // 速度ループ比例ゲイン
  eSpeed_loop_integral_gain,         // 速度ループ積分ゲイン
  eCurrent_position_clear,           // 現在位置クリアコマンド
  eLock_shaft_stop_noise_reduction_enabled,  // ロックシャフトストップノイズリダクションイネーブル
  // ロックシャフトストップノイズリダクション効果遅延
  eLock_shaft_stop_noise_reduction_effect_delay,
  eSpring_compensation_coefficient,  // ばね補償係数
  eSpring_compensation_base_point,   // ばね補償基点
  eDriver_overheat_alarm_point,      // ドライバー過熱アラームポイント
  eActual_position_value,            // 実際の位置の値
  eActual_speed_rpm,                 // 実際の速度rpm
  eActual_speed_0001rpm,             // 実際の速度 0.001rpm
  eActual_speed_sampling_period,     // 実際の速度のサンプリング期間
  eActual_speed,                     // 実際の速度
  eActual_current_IQ,                // 実際の現在のIq
  eMotor_Iit_actual_current,         // モーターIit実際の電流
  eActual_DC_bus_voltage,            // 実際のDCバス電圧
  eActual_temperature_of_the_drive,  // ドライブの実際の温度
  ePosition_after_error,             // エラー後の位置
  // デジタル入力ポート（ハードウェア入力）の実際の入力状態
  eActual_input_state_of_digital_input_port,
  eDigital_input_port_polarity_setting,  // デジタル入力ポートの極性設定
  // デジタル入力ポートシミュレーション入力（ソフトウェアシミュレーション入力状態）
  eDigital_input_port_Simulation_input,
  eVirtual_state_of_digital_input,  // デジタル入力の仮想状態
  eTTL232_baud_rate_setting,        // TTL232ボーレート設定
  eTTL232_protocol_selection,       // TTL232プロトコルの選択
  eRS485_CAN_station_number,  // RS485/CANステーション番号（内部ストレージ）
  eDrop_and_shutdown,         // 通信のドロップとシャットダウンを有効にする
  eDowntime_and_shutdown_delay,  // 通信のダウンタイムとシャットダウンの遅延
  eSimple_PDO_function,          // シンプルなPDO機能
  eTX_PDO1_invalidation_time,    // TX-PDO1の無効化時間。
  eS1_parameter_save,            // S1パラメータグループの保存コマンド
  eS2_parameter_save,            // S2パラメータグループの保存コマンド
  eS3_parameter_save,            // S3パラメータグループの保存コマンド
  eS4_parameter_save,            // S4パラメータグループの保存コマンド
  eS5_parameter_save,            // S5パラメータグループの保存コマンド
  eModel_series,                 // このマシンモデルシリーズ
  eModel_voltage_and_current_level,  // このマシンモデル-電圧および電流レベル
  eModel_feedback_and_bus_type,  // ネイティブモデル-フィードバックとバスタイプ
  eModel_expression,             // ネイティブモデル-式
  eFirmware_date,                // ファームウェアの日付
  eHardware_version,             // ハードウェアバージョン
  eSstop_function_is_enabled,    // Sstop機能が有効
  eStop_current_limit,           // 停止電流制限値
  eDecelerating_speed_point_S_curve,  // 減速時にSカーブに入る速度点
  eDecelerating_time_point_S_curve,   // 減速時にSカーブが重なる時間の長さ
};

struct Spec
{
  uint32_t serial_cmd_id;      // シリアル通信コマンドID
  uint32_t can_cmd_id;         // CAN通信コマンドID
  std::string type;        // データタイプ
  std::string write_size;  // 書き込みサイズ
  std::string read_size;   // 読み込みサイズ
};

std::map<CommandID, Spec> spec_map = {
  //  key                                                     value
  {CommandID::eCurrent_motor_model_code,
    {0x7046, 0x641016, "RO", "16U", "16U"}},  // 現在のモーターモデルコード
  {CommandID::eMotor_model_code, {0x7031, 0x641001, "RW", "16U", "04S"}},  // モーターモデルコード
  {CommandID::eFeedback_type, {0x7032, 0x641002, "RW", "08U", "04S"}},  // フィードバックタイプ
  {CommandID::eEncoder_resolution, {0x7033, 0x641003, "RW", "32U", "04S"}},  // エンコーダの分解能
  {CommandID::eNumber_of_motor_pole_pairs,
    {0x7035, 0x641005, "RW", "08U", "04S"}},  // モーターポールペアの数
  {CommandID::eExcitation_mode, {0x7036, 0x641006, "RW", "08U", "04S"}},     // 励起モード
  {CommandID::eExcitation_current, {0x7037, 0x641007, "RW", "16S", "04S"}},  // 励起電流
  {CommandID::eExcitation_time, {0x7038, 0x641008, "RW", "16U", "04S"}},     // 励起時間
  {CommandID::eMotor_overload_current,
    {0x7039, 0x641009, "RW", "16U", "04S"}},  // モーター過負荷電流
  {CommandID::eMotor_overload_time_constant,
    {0x703A, 0x64100A, "RW", "16U", "04S"}},  // モーター過負荷時定数
  {CommandID::eMaximum_motor_current,
    {0x703B, 0x64100B, "RW", "16U", "04S"}},  // 最大モーター電流
  {CommandID::eMotor_rotation_direction,
    {0x7043, 0x641013, "RW", "08U", "04S"}},  // モーターの回転方向
  {CommandID::eMotor_rated_speed, {0x704A, 0x64101A, "RW", "16U", "04S"}},  // モーター定格速度
  {CommandID::eMotor_rated_power, {0x704B, 0x64101B, "RW", "16U", "04S"}},  // モーター定格電力
  {CommandID::eMotor_hole_angle, {0x704F, 0x64101F, "RW", "16S", "04S"}},  // モーターホール角
  {CommandID::eAction_mode, {0x7017, 0x606000, "RW", "08S", "05S"}},       // 動作モード
  {CommandID::eControl_word, {0x7019, 0x604000, "RW", "16S", "16S"}},      // 制御語
  {CommandID::eEffective_Action_mode,
    {0x7018, 0x606100, "RO", "08S", "08S"}},  // 有効な動作モード
  {CommandID::eDrive_status_word,
    {0x7001, 0x604100, "RO", "16U", "16U"}},  // ドライブステータスワード
  {CommandID::eDrive_error_status_word_1,
    {0x7011, 0x260100, "RO", "16U", "16U"}},  // ドライブエラーステータスワード1
  {CommandID::eEmergency_stop_order, {0x701F, 0x605A11, "RW", "08U", "08U"}},  // 非常停止命令
  {CommandID::eAbsolute_target_position, {0x7091, 0x607A00, "RW", "32S", "32S"}},  // 絶対目標位置
  {CommandID::eRelative_target_position, {0x709F, 0x607B00, "RW", "32S", "32S"}},  // 相対目標位置
  {CommandID::eTrapezoidal_speed, {0x7098, 0x608100, "RW", "32U", "05S"}},      // 台形の速度
  {CommandID::eTrapezoidal_speed_rpm, {0x709D, 0x608200, "RW", "16U", "05S"}},  // 台形速度rpm
  {CommandID::eTrapezoidal_acceleration, {0x7099, 0x608300, "RW", "32U", "05S"}},  // 台形加速度
  {CommandID::eTrapezoidal_deceleration, {0x709A, 0x608400, "RW", "32U", "05S"}},  // 台形減速
  {CommandID::eSudden_stop_deceleration, {0x709B, 0x605A01, "RW", "32U", "05S"}},  // 急停止減速
  {CommandID::eTarget_speed_rpm, {0x70B1, 0x2FF009, "RW", "16S", "16S"}},      // 目標速度rpm
  {CommandID::eTarget_speed, {0x70B2, 0x60FF00, "RW", "32S", "32S"}},          // 目標速度
  {CommandID::eTarget_current, {0x70E1, 0x60F608, "RW", "16S", "16S"}},        // 目標電流
  {CommandID::eTarget_current_limit, {0x70E2, 0x607300, "RW", "16U", "16U"}},  // 目標電流制限
  {CommandID::eMaximum_speed_limit_rpm,
    {0x70B8, 0x608000, "RW", "16U", "05S"}},  // 最高速度制限rpm
  {CommandID::ePosition_error_window,
    {0x7093, 0x606500, "RW", "32U", "05S"}},  // エラーウィンドウに続く位置
  {CommandID::eWork_mode_3, {0x8088, 0x60FB88, "RW", "08U", "05S"}},  // 作業モード3
  {CommandID::ePosition_loop_speed_feedforward,
    {0x709C, 0x60FB02, "RW", "16S", "05S"}},  // 位置ループ速度フィードフォワード
  {CommandID::ePosition_loop_proportional_gain,
    {0x7094, 0x60FB01, "RW", "16S", "05S"}},  // 位置ループ比例ゲイン
  {CommandID::eSpeed_loop_proportional_gain,
    {0x70B3, 0x60F901, "RW", "16U", "05S"}},  // 速度ループ比例ゲイン
  {CommandID::eSpeed_loop_integral_gain,
    {0x70B4, 0x60F902, "RW", "16U", "05S"}},  // 速度ループ積分ゲイン
  {CommandID::eCurrent_position_clear,
    {0x70AC, 0x607C02, "RW", "08U", "08U"}},  // 現在位置クリアコマンド
  {CommandID::eLock_shaft_stop_noise_reduction_enabled,
    {0x701C, 0x500050, "RW", "08U", "05S"}},  // ロックシャフトストップノイズリダクションイネーブル
  {CommandID::eLock_shaft_stop_noise_reduction_effect_delay,
    {0x701D, 0x500051, "RW", "16U", "05S"}},  // ロックシャフトストップノイズリダクション効果遅延
  {CommandID::eSpring_compensation_coefficient,
    {0x80BC, 0x60F98C, "RW", "16U", "05S"}},  // ばね補償係数
  {CommandID::eSpring_compensation_base_point,
    {0x80BE, 0x60F98E, "RW", "16U", "05S"}},  // ばね補償基点
  {CommandID::eDriver_overheat_alarm_point,
    {0x7004, 0x300015, "RW", "16U", "S3"}},  // ドライバー過熱アラームポイント
  {CommandID::eActual_position_value, {0x7071, 0x606300, "RO", "32S", "32S"}},  // 実際の位置の値
  {CommandID::eActual_speed_rpm, {0x7075, 0x60F918, "RO", "16S", "16S"}},  // 実際の速度rpm
  {CommandID::eActual_speed_0001rpm,
    {0x7076, 0x60F919, "RO", "32S", "32S"}},  // 実際の速度 0.001rpm
  {CommandID::eActual_speed_sampling_period,
    {0x7079, 0x60F91A, "RW", "16U", "05S"}},  // 実際の速度のサンプリング期間
  {CommandID::eActual_speed, {0x7077, 0x606C00, "RO", "32S", "32S"}},       // 実際の速度
  {CommandID::eActual_current_IQ, {0x7072, 0x607800, "RO", "16S", "16S"}},  // 実際の現在のIq
  {CommandID::eMotor_Iit_actual_current,
    {0x7007, 0x60F632, "RO", "16U", "16U"}},  // モーターIit実際の電流
  {CommandID::eActual_DC_bus_voltage,
    {0x5001, 0x60F712, "RO", "16S", "16S"}},  // 実際のDCバス電圧
  {CommandID::eActual_temperature_of_the_drive,
    {0x7002, 0x60F70B, "RO", "16S", "16S"}},  // ドライブの実際の温度
  {CommandID::ePosition_after_error, {0x7092, 0x60F400, "RO", "32S", "32S"}},  // エラー後の位置
  {CommandID::eActual_input_state_of_digital_input_port,
    {0x5100, 0x20100A, "RO", "16U",
    "16U"}},  // デジタル入力ポート（ハードウェア入力）の実際の入力状態
  {CommandID::eDigital_input_port_polarity_setting,
    {0x5100, 0x20100A, "RO", "16U", "02S"}},  // デジタル入力ポートの極性設定
  {CommandID::eDigital_input_port_Simulation_input,
    {0x5101, 0x201002, "RW", "16U",
    "16U"}},  // デジタル入力ポートシミュレーション入力（ソフトウェアシミュレーション入力状態）
  {CommandID::eVirtual_state_of_digital_input,
    {0x5103, 0x20100B, "RO", "16U", "16U"}},  // デジタル入力の仮想状態
  {CommandID::eTTL232_baud_rate_setting,
    {0x1005, 0x2FE001, "RW", "08U", "01S"}},  // TTL232ボーレート設定
  {CommandID::eTTL232_protocol_selection,
    {0x100F, 0x2FE00F, "RW", "08U", "01S"}},  // TTL232プロトコルの選択
  {CommandID::eRS485_CAN_station_number,
    {0x100C, 0x65100F, "RW", "08U", "01S"}},  // RS485/CANステーション番号（内部ストレージ）
  {CommandID::eDrop_and_shutdown,
    {0x3010, 0x410010, "RW", "08U", "01S"}},  // 通信のドロップとシャットダウンを有効にする
  {CommandID::eDowntime_and_shutdown_delay,
    {0x3011, 0x410011, "RW", "32U", "01S"}},  // 通信のダウンタイムとシャットダウンの遅延
  {CommandID::eSimple_PDO_function, {0x0000, 0x470001, "RW", "08U", "08U"}},  // シンプルなPDO機能
  {CommandID::eTX_PDO1_invalidation_time,
    {0x0000, 0x180003, "RW", "16U", "16U"}},  // TX-PDO1の無効化時間。
  {CommandID::eS1_parameter_save,
    {0x3061, 0x2FE501, "RW", "08U", "08U"}},  // S1パラメータグループの保存コマンド
  {CommandID::eS2_parameter_save,
    {0x3062, 0x2FE502, "RW", "08U", "08U"}},  // S2パラメータグループの保存コマンド
  {CommandID::eS3_parameter_save,
    {0x3063, 0x2FE503, "RW", "08U", "08U"}},  // S3パラメータグループの保存コマンド
  {CommandID::eS4_parameter_save,
    {0x3064, 0x2FE504, "RW", "08U", "08U"}},  // S4パラメータグループの保存コマンド
  {CommandID::eS5_parameter_save,
    {0x3065, 0x2FE505, "RW", "08U", "08U"}},  // S5パラメータグループの保存コマンド
  {CommandID::eModel_series, {0x103A, 0x30003A, "RO", "32U", "32U"}},  // このマシンモデルシリーズ
  {CommandID::eModel_voltage_and_current_level,
    {0x103B, 0x30003B, "RO", "32U", "32U"}},  // このマシンモデル-電圧および電流レベル
  {CommandID::eModel_feedback_and_bus_type,
    {0x103C, 0x30003C, "RO", "32U", "32U"}},  // ネイティブモデル-フィードバックとバスタイプ
  {CommandID::eModel_expression, {0x103D, 0x30003D, "RO", "32U", "32U"}},  // ネイティブモデル-式
  {CommandID::eFirmware_date, {0x1007, 0x2500F1, "RO", "32U", "32U"}},  // ファームウェアの日付
  {CommandID::eHardware_version,
    {0x1009, 0x2500F3, "RO", "32U", "32U"}},  // ハードウェアバージョン
  {CommandID::eSstop_function_is_enabled,
    {0x802F, 0x500020, "RW", "08U", "05S"}},  // Sstop機能が有効
  {CommandID::eStop_current_limit, {0x8028, 0x500028, "RW", "32U", "05S"}},  // 停止電流制限値
  {CommandID::eDecelerating_speed_point_S_curve,
    {0x8002, 0x500002, "RW", "32U", "05S"}},  // 減速時にSカーブに入る速度点
  {CommandID::eDecelerating_time_point_S_curve,
    {0x8004, 0x500004, "RW", "16U", "05S"}},  // 減速時にSカーブが重なる時間の長さ
};

enum class ComType : int32_t {
  read,
  write,
};

const uint8_t SendingCmd_ReadData = 0x40;
const uint8_t SendingCmd_Write1Byte = 0x2F;
const uint8_t SsndingCmd_Write2Bytes = 0x2B;
const uint8_t SendingCmd_Write4Bytes = 0x23;
const uint8_t ReceivingCmd_WriteSuccess = 0x60;
const uint8_t ReceivingCmd_WriteFaild = 0x80;
const uint8_t ReceivingCmd_Read1Byte = 0x4F;
const uint8_t ReceivingCmd_Read2Bytes = 0x4B;
const uint8_t ReceivingCmd_Read4Bytes = 0x43;

}   // namespace lsdb_command_list


#endif  // LSDB_CAN_INTERFACE__LSDB_COMMAND_LIST_HPP_
