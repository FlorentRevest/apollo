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

#ifndef MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_THROTTLE_DISABLE_91_H_
#define MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_THROTTLE_DISABLE_91_H_

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace niro {

class Throttledisable91 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Throttledisable91();

  void UpdateData(uint8_t* data) override;

  void Reset() override;

 private:
  // config detail: {'name': 'throttle_disable_magic', 'enum': {1372: 'THROTTLE_DISABLE_MAGIC_DEFAULT_MAGIC_NUMBER'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_throttle_disable_magic(uint8_t* data);
};

}  // namespace niro
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_NIRO_PROTOCOL_THROTTLE_DISABLE_91_H_
