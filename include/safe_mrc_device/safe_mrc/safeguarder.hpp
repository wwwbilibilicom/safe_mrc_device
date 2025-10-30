/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

// #include <safe_mrc_device/rs485bus/rs485_serial.hpp>
#include <string>
#include <vector>

#include "safe_mrc_device/rs485bus/rs485_device_collection.hpp"
#include "safe_mrc_device/rs485bus/rs485_serial.hpp"
#include "safe_mrc_device/safe_mrc/safe_mrc_component.hpp"

namespace safe_mrc {

class Safeguarder {
 public:
  Safeguarder(const std::string& port);
  ~Safeguarder() = default;

  void init_mrcs(const std::vector<MRCType>& mrc_types,
                 const std::vector<uint8_t>& rs485_ids);

  MRCComponent& get_mrc_component() { return *mrc_component_; }
  RS485DeviceCollection& get_master_rs485_device_collection() {
    return *master_rs485_device_collection_;
  }

  void enable_all();
  void disable_all();
  void refresh_all_commands();
  void set_zero_all();
  int refresh_all_states();

 private:
  std::string port_;
  std::unique_ptr<RS485Serial> rs485_serial_;
  std::unique_ptr<MRCComponent> mrc_component_;
  std::unique_ptr<RS485DeviceCollection> master_rs485_device_collection_;
  std::vector<SafeMRCDeviceCollection*> sub_mrc_device_collections_;
  void register_mrc_device_collection(
      SafeMRCDeviceCollection& device_collection);
};

}  // namespace safe_mrc
