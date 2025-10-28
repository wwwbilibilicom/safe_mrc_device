/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

#include <memory>
#include <vector>
#include "safe_mrc_device/safe_mrc/safe_mrc.hpp"
#include "safe_mrc_device/safe_mrc/safe_mrc_device_collection.hpp"

namespace safe_mrc {

class MRCComponent : public SafeMRCDeviceCollection {
 public:
  MRCComponent(RS485Serial& rs485_serial,
               std::shared_ptr<BusDiagnostics> diagnostics = nullptr);
  ~MRCComponent() = default;

  void init_mrc_devices(std::vector<MRCType> mrc_types,
                        std::vector<uint8_t> rs485_ids);

 private:
  std::vector<MRC> mrcs_;
};

}  // namespace safe_mrc
