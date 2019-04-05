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

#include "modules/canbus/vehicle/niro/protocol/steering_angle_2b0.h"


#include "glog/logging.h"

#include "modules/canbus/common/byte.h"
#include "modules/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace niro {

Steeringangle2b0::Steeringangle2b0() {}
const int32_t Steeringangle2b0::ID = 0x2B0;

void Steeringangle2b0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_eps()->set_steering_angle(steering_wheel_angle(bytes, length));
  chassis->mutable_eps()->set_is_steering_angle_valid(true);
}

// config detail: {'name': 'steering_wheel_angle', 'offset': 0.0, 'precision': 0.018, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'degrees'}
double Steeringangle2b0::steering_wheel_angle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return -(x * 0.021);
}
}  // namespace niro
}  // namespace canbus
}  // namespace apollo
