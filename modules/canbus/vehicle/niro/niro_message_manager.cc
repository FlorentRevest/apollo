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

#include "modules/canbus/vehicle/niro/niro_message_manager.h"

#include "modules/canbus/vehicle/niro/protocol/brake_command_72.h"
#include "modules/canbus/vehicle/niro/protocol/brake_disable_71.h"
#include "modules/canbus/vehicle/niro/protocol/brake_enable_70.h"
#include "modules/canbus/vehicle/niro/protocol/steering_disable_81.h"
#include "modules/canbus/vehicle/niro/protocol/steering_enable_80.h"
#include "modules/canbus/vehicle/niro/protocol/steering_torque_command_82.h"
#include "modules/canbus/vehicle/niro/protocol/throttle_command_92.h"
#include "modules/canbus/vehicle/niro/protocol/throttle_disable_91.h"
#include "modules/canbus/vehicle/niro/protocol/throttle_enable_90.h"

#include "modules/canbus/vehicle/niro/protocol/brake_pressure_220.h"
#include "modules/canbus/vehicle/niro/protocol/speed_52a.h"
#include "modules/canbus/vehicle/niro/protocol/steering_angle_2b0.h"
#include "modules/canbus/vehicle/niro/protocol/throttle_pressure_371.h"

namespace apollo {
namespace canbus {
namespace niro {

NiroMessageManager::NiroMessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecommand72, true>();
  AddSendProtocolData<Brakedisable71, true>();
  AddSendProtocolData<Brakeenable70, true>();
  AddSendProtocolData<Steeringtorquecommand82, true>();
  AddSendProtocolData<Steeringdisable81, true>();
  AddSendProtocolData<Steeringenable80, true>();
  AddSendProtocolData<Throttlecommand92, true>();
  AddSendProtocolData<Throttledisable91, true>();
  AddSendProtocolData<Throttleenable90, true>();

  // Report Messages
  AddRecvProtocolData<Brakepressure220, true>();
  AddRecvProtocolData<Speed52a, true>();
  AddRecvProtocolData<Steeringangle2b0, true>();
  AddRecvProtocolData<Throttlepressure371, true>();
}

NiroMessageManager::~NiroMessageManager() {}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
