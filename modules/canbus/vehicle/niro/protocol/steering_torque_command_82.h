/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_STEERING_TORQUE_COMMAND_82_H_
#define MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_STEERING_TORQUE_COMMAND_82_H_

#include "modules/canbus/vehicle/protocol_data.h"
#include "modules/canbus/vehicle/message_manager.h"
//#include "modules/canbus/vehicle/vehicle_controller.h"

namespace apollo {
namespace canbus {
namespace niro {

class Steeringtorquecommand82 : public ProtocolData {
 public:
  static const int32_t ID;

  Steeringtorquecommand82();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Steeringtorquecommand82* set_steering_angle_target(double angle_target);

  void set_message_manager(MessageManager * message_manager);
//  void set_controller(VehicleController * controller);

 private:

  // config detail: {'name': 'steering_command_magic', 'enum': {1372: 'STEERING_COMMAND_MAGIC_DEFAULT_MAGIC_NUMBER'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_command_magic(uint8_t* data);

  // config detail: {'name': 'steering_torque_command', 'offset': 0.0, 'precision': 1.0, 'len': 32, 'is_signed_var': True, 'physical_range': '[-1|1]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_torque_command(uint8_t* data, double angle_target);

 private:
  double angle_target_;
  double torque_command_average_;
  MessageManager * message_manager_;
//  VehicleController * controller_;
  double integral_, pre_error_;
};

}  // namespace niro
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_NIRO_PROTOCOL_STEERING_TORQUE_COMMAND_82_H_
