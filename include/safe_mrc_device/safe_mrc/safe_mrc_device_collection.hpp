/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "safe_mrc_device/rs485bus/rs485_device_collection.hpp"
#include "safe_mrc_device/safe_mrc/safe_mrc.hpp"
#include "safe_mrc_device/safe_mrc/safe_mrc_device.hpp"

namespace safe_mrc {
class SafeMRCDeviceCollection {
 public:
  SafeMRCDeviceCollection(RS485Serial& rs485_serial);
  virtual ~SafeMRCDeviceCollection() = default;

  void enable_all();
  void disable_all();
  void set_zero_all();

  void refresh_one(int i);
  void refresh_all();

  void mrc_control_one(int i, const MRCCmd& cmd);
  void mrc_control_all(const std::vector<MRCCmd>& cmds);

  std::vector<MRC> get_mrcs() const;
  RS485DeviceCollection& get_device_collection() { return *device_collection_; }

 protected:
  RS485Serial& rs485_serial_;
  std::unique_ptr<RS485DeviceCollection> device_collection_;

  void send_command_to_device(std::shared_ptr<SafeMRCRS485Device> device,
                              const MRCCmd& cmd);

  std::vector<std::shared_ptr<SafeMRCRS485Device>> get_safe_mrc_devices() const;

  uint8_t rx_delay_us_;
};
}  // namespace safe_mrc
