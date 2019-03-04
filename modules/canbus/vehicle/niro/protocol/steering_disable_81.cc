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

#include "modules/canbus/vehicle/niro/protocol/steering_disable_81.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace niro {

const int32_t Steeringdisable81::ID = 0x81;

// public
Steeringdisable81::Steeringdisable81() { Reset(); }

void Steeringdisable81::UpdateData(uint8_t* data) {
  set_p_steering_disable_magic(data);
}

void Steeringdisable81::Reset() { }

// config detail: {'name': 'steering_disable_magic', 'enum': {1372: 'STEERING_DISABLE_MAGIC_DEFAULT_MAGIC_NUMBER'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Steeringdisable81::set_p_steering_disable_magic(uint8_t* data) {
  Byte to_set0(data + 0);
  to_set0.set_value(0x05, 0, 8);

  Byte to_set1(data + 1);
  to_set1.set_value(0xCC, 0, 8);
}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
