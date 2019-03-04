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

#include "modules/canbus/vehicle/niro/protocol/throttle_enable_90.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace niro {

const int32_t Throttleenable90::ID = 0x90;

// public
Throttleenable90::Throttleenable90() { Reset(); }

void Throttleenable90::UpdateData(uint8_t* data) {
  set_p_throttle_enable_magic(data);
}

void Throttleenable90::Reset() { }

// config detail: {'name': 'throttle_enable_magic', 'enum': {1372: 'THROTTLE_ENABLE_MAGIC_DEFAULT_MAGIC_NUMBER'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Throttleenable90::set_p_throttle_enable_magic(uint8_t* data) {
  Byte to_set0(data + 0);
  to_set0.set_value(0x05, 0, 8);

  Byte to_set1(data + 1);
  to_set1.set_value(0xCC, 0, 8);
}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
