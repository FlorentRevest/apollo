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

#include "modules/canbus/vehicle/niro/protocol/speed_52a.h"


#include "glog/logging.h"

#include "modules/canbus/common/byte.h"
#include "modules/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace niro {

Speed52a::Speed52a() {}
const int32_t Speed52a::ID = 0x52A;

void Speed52a::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_eps()->set_vehicle_speed(speed(bytes, length));
  chassis->mutable_vehicle_spd()->set_vehicle_spd(speed(bytes, length));
  chassis->mutable_vehicle_spd()->set_is_vehicle_spd_valid(true);
}

// config detail: {'name': 'speed', 'offset': 0.0, 'precision': 0.3, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'mps'}
double Speed52a::speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.3;
  return ret;
}
}  // namespace niro
}  // namespace canbus
}  // namespace apollo
