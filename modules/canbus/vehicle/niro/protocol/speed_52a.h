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

#ifndef MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_SPEED_52A_H_
#define MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_SPEED_52A_H_

#include "modules/canbus/vehicle/protocol_data.h"

namespace apollo {
namespace canbus {
namespace niro {

class Speed52a : public ProtocolData {
 public:
  static const int32_t ID;
  Speed52a();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SPEED', 'offset': 0.0, 'precision': 0.3, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'mps'}
  double speed(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace niro
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_NIRO_PROTOCOL_SPEED_52A_H_
