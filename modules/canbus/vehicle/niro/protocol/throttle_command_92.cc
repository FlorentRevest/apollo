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

#include "modules/canbus/vehicle/niro/protocol/throttle_command_92.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace niro {

const int32_t Throttlecommand92::ID = 0x92;

// public
Throttlecommand92::Throttlecommand92() { Reset(); }

uint32_t Throttlecommand92::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Throttlecommand92::UpdateData(uint8_t* data) {
  set_p_throttle_command_magic(data);
  set_p_throttle_pedal_command(data, throttle_pedal_command_);
}

void Throttlecommand92::Reset() {
  throttle_pedal_command_ = 0;
}

// config detail: {'name': 'throttle_command_magic', 'enum': {1372: 'THROTTLE_COMMAND_MAGIC_DEFAULT_MAGIC_NUMBER'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Throttlecommand92::set_p_throttle_command_magic(uint8_t* data) {
  Byte to_set0(data + 0);
  to_set0.set_value(0x05, 0, 8);

  Byte to_set1(data + 1);
  to_set1.set_value(0xCC, 0, 8);
}

Throttlecommand92* Throttlecommand92::set_throttle_pedal_command(
    double throttle_pedal_command) {
  throttle_pedal_command_ = throttle_pedal_command;
  return this;
 }

// config detail: {'name': 'throttle_pedal_command', 'offset': 0.0, 'precision': 1.0, 'len': 32, 'is_signed_var': True, 'physical_range': '[0|1]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Throttlecommand92::set_p_throttle_pedal_command(uint8_t* data,
    double throttle_pedal_command) {
  float throttle_pedal_command_f = ProtocolData::BoundedValue(0.0, 1.0, throttle_pedal_command);
  unsigned int x = *((unsigned int*)&throttle_pedal_command_f);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set2(data + 4);
  to_set2.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set3(data + 5);
  to_set3.set_value(t, 0, 8);
}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
