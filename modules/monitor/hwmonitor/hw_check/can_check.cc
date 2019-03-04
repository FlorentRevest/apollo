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

#include <assert.h>
#include <iostream>

#include "glog/logging.h"
#include "modules/hmi/utils/hmi_status_helper.h"
#include "modules/monitor/common/annotations.h"
#include "modules/monitor/common/log.h"
#include "modules/monitor/common/annotations.h"
#include "modules/monitor/common/interface/hw_checker.h"
#include "modules/monitor/hwmonitor/hw/hw_log_module.h"
#include "modules/monitor/hwmonitor/hw_check/hw_chk_utils.h"

using apollo::platform::HwCheckResult;
using apollo::hmi::HMIStatusHelper;
using apollo::hmi::HardwareStatus;

int main(int argc, const char *argv[]) {
  // For other modules that uses glog.
  ::google::InitGoogleLogging("platform");
  apollo::platform::log::init_syslog();
  // @todo make log level configurable or set lower level here.
#ifdef DEBUG
  apollo::platform::hw::config_log(apollo::platform::log::LVL_DBG,
                                   apollo::platform::log::DBG_VERBOSE,
                                   apollo::platform::log::platform_log_printf);
#else
  apollo::platform::hw::config_log(apollo::platform::log::LVL_DBG,
                                   apollo::platform::log::DBG_VERBOSE);
#endif

  // We only have can0 for now.
  std::vector<HwCheckResult> can_rslt;
  HwCheckResult rslt("ESD_CAN-0", apollo::platform::hw::Status::OK);
  can_rslt.emplace_back(std::move(rslt));

  std::vector<HardwareStatus> hw_status;
  apollo::platform::hw::hw_chk_result_to_hmi_status(can_rslt, &hw_status);

  HMIStatusHelper::ReportHardwareStatus(hw_status);

  return 0;
}
