/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/niro/niro_controller.h"

#include "modules/common/log.h"

#include "modules/canbus/can_comm/can_sender.h"
#include "modules/canbus/vehicle/niro/niro_message_manager.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace canbus {
namespace niro {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode NiroController::Init(const VehicleParameter& params,
                                  CanSender* const can_sender,
                                  MessageManager* const message_manager) {
  if (is_initialized_) {
    AINFO << "NiroController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  brake_command_72_ = dynamic_cast<Brakecommand72*>
          (message_manager_->GetMutableProtocolDataById(Brakecommand72::ID));
  if (brake_command_72_ == nullptr) {
     AERROR << "Brakecommand72 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  brake_disable_71_ = dynamic_cast<Brakedisable71*>
          (message_manager_->GetMutableProtocolDataById(Brakedisable71::ID));
  if (brake_disable_71_ == nullptr) {
     AERROR << "Brakedisable71 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  brake_enable_70_ = dynamic_cast<Brakeenable70*>
          (message_manager_->GetMutableProtocolDataById(Brakeenable70::ID));
  if (brake_enable_70_ == nullptr) {
     AERROR << "Brakeenable70 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  steering_torque_command_82_ = dynamic_cast<Steeringtorquecommand82*>
          (message_manager_->GetMutableProtocolDataById(Steeringtorquecommand82::ID));
  if (steering_torque_command_82_ == nullptr) {
     AERROR << "Steeringtorquecommand82 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }
  steering_torque_command_82_->set_message_manager(message_manager);
  //steering_torque_command_82_->set_controller(this);

  steering_disable_81_ = dynamic_cast<Steeringdisable81*>
          (message_manager_->GetMutableProtocolDataById(Steeringdisable81::ID));
  if (steering_disable_81_ == nullptr) {
     AERROR << "Steeringdisable81 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  steering_enable_80_ = dynamic_cast<Steeringenable80*>
          (message_manager_->GetMutableProtocolDataById(Steeringenable80::ID));
  if (steering_enable_80_ == nullptr) {
     AERROR << "Steeringenable80 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  throttle_command_92_ = dynamic_cast<Throttlecommand92*>
          (message_manager_->GetMutableProtocolDataById(Throttlecommand92::ID));
  if (throttle_command_92_ == nullptr) {
     AERROR << "Throttlecommand92 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  throttle_disable_91_ = dynamic_cast<Throttledisable91*>
          (message_manager_->GetMutableProtocolDataById(Throttledisable91::ID));
  if (throttle_disable_91_ == nullptr) {
     AERROR << "Throttledisable91 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  throttle_enable_90_ = dynamic_cast<Throttleenable90*>
          (message_manager_->GetMutableProtocolDataById(Throttleenable90::ID));
  if (throttle_enable_90_ == nullptr) {
     AERROR << "Throttleenable90 does not exist in the NiroMessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecommand72::ID, brake_command_72_, false);
  can_sender_->AddMessage(Steeringtorquecommand82::ID, steering_torque_command_82_, false);
  can_sender_->AddMessage(Throttlecommand92::ID, throttle_command_92_, false);

  // need sleep to ensure all messages received
  AINFO << "NiroController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

NiroController::~NiroController() {}

bool NiroController::Start() {
  if (!is_initialized_) {
    AERROR << "NiroController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void NiroController::Stop() {
  if (!is_initialized_) {
    AERROR << "NiroController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "NiroController stopped.";
  }
}

Chassis NiroController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetChassisDetail(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // TODO: Is there anything we want to store after here ?

  // 4
  chassis_.set_engine_rpm(0);

  // 5
  if (chassis_detail.vehicle_spd().has_vehicle_spd()) {
    chassis_.set_speed_mps(chassis_detail.vehicle_spd().vehicle_spd());
  } else {
    chassis_.set_speed_mps(0);
  }
  
  // 6
  chassis_.set_odometer_m(0);

  // 7
  chassis_.set_fuel_range_m(0);

  // 8
  if (chassis_detail.gas().has_throttle_output()) {
    chassis_.set_throttle_percentage(chassis_detail.gas().throttle_output());
  } else {
    chassis_.set_throttle_percentage(0);
  }

  // 9
  if (chassis_detail.brake().has_brake_output()) {
    chassis_.set_brake_percentage(chassis_detail.brake().brake_output());
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 23, previously 10
  chassis_.set_gear_location(Chassis::GEAR_NONE);

  // 11
  if (chassis_detail.eps().has_steering_angle()) {
    chassis_.set_steering_percentage(chassis_detail.eps().steering_angle());
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 12
  chassis_.set_steering_torque_nm(0);

  // 13
  chassis_.set_parking_brake(false);

  // 14, 15
  // beams

  // 16, 17
  chassis_.mutable_signal()->set_turn_signal(Signal::TURN_NONE);

  // 18
  chassis_.mutable_signal()->set_horn(false);

  return chassis_;
}

void NiroController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode NiroController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  can_sender_->SendMessageOnce(Brakeenable70::ID, brake_enable_70_);
  can_sender_->SendMessageOnce(Throttleenable90::ID, throttle_enable_90_);
  can_sender_->SendMessageOnce(Steeringenable80::ID, steering_enable_80_);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode NiroController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode NiroController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }

  can_sender_->SendMessageOnce(Brakedisable71::ID, brake_disable_71_);
  can_sender_->SendMessageOnce(Throttledisable91::ID, throttle_disable_91_);
  can_sender_->SendMessageOnce(Steeringenable80::ID, steering_enable_80_);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode NiroController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }

  can_sender_->SendMessageOnce(Brakeenable70::ID, brake_enable_70_);
  can_sender_->SendMessageOnce(Throttleenable90::ID, throttle_enable_90_);
  can_sender_->SendMessageOnce(Steeringdisable81::ID, steering_disable_81_);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

// NEUTRAL, REVERSE, DRIVE
void NiroController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }

  // TODO: Do we have any sort of control over the gear position?
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void NiroController::Brake(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }

  const double real_pedal = pedal / 100.0;
  brake_command_72_->set_brake_pedal_command(real_pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void NiroController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }

  const double real_pedal = pedal / 100.0;
  throttle_command_92_->set_throttle_pedal_command(real_pedal);
}

// set the targeted steering angle
// angle:        -99.99~0.00~99.99, unit:percentage, left:-, right:+
void NiroController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  steering_torque_command_82_->set_steering_angle_target(angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void NiroController::Steer(double angle, double angle_spd) {
  // TODO: Can we use angle_spd at all?
  Steer(angle);
}

void NiroController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void NiroController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // TODO: Secondary CAN
  } else if (command.signal().low_beam()) {
    // TODO: Secondary CAN
  } else {
    // TODO: Secondary CAN
  }
}

void NiroController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // TODO: Secondary CAN
  } else {
    // TODO: Secondary CAN
  }
}

void NiroController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    // TODO: Secondary CAN
  } else if (signal == Signal::TURN_RIGHT) {
    // TODO: Secondary CAN
  } else {
    // TODO: Secondary CAN
  }
}

void NiroController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool NiroController::CheckChassisError() {
  // TODO: Is there anything we want to check ? (watchdogs, connectors, faults)
  return false;
}

void NiroController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in NiroController looping process:"
          << elapsed.count();
    }
  }
}

bool NiroController::CheckResponse(const int32_t flags, bool need_wait) {
  // TODO: Is there anything we want to check ? (eps, vcu, esp)
  return true;
}

void NiroController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t NiroController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode NiroController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void NiroController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
