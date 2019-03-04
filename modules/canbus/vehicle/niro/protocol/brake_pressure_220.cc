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

#include "modules/canbus/vehicle/niro/protocol/brake_pressure_220.h"


#include "glog/logging.h"

#include "modules/canbus/common/byte.h"
#include "modules/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace niro {

Brakepressure220::Brakepressure220() {}
const int32_t Brakepressure220::ID = 0x220;

void Brakepressure220::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_brake()->set_brake_output(brake_pressure(bytes, length));
}

// config detail: {'name': 'brake_pressure', 'offset': 0.0, 'precision': 0.033, 'len': 16 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'double', 'order': 'intel', 'physical_unit': 'bar'}
double Brakepressure220::brake_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.033;
  return ret;
}
}  // namespace niro
}  // namespace canbus
}  // namespace apollo
