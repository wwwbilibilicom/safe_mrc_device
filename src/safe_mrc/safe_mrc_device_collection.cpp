/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/safe_mrc/safe_mrc_device_collection.hpp"
#include <cstring>
#include <iostream>

namespace safe_mrc {
SafeMRCDeviceCollection::SafeMRCDeviceCollection(RS485Serial& rs485_serial)
    : rs485_serial_(rs485_serial),
      device_collection_(
          std::make_unique<RS485DeviceColllection>(rs485_serial_)) {}

void SafeMRCDeviceCollection::enable_all() {
  for (auto device : get_safe_mrc_devices()) {
    send_command_to_device(device, MRCCmd{MRCMode::ADAPTATION, 0.0f});
  }
}

void SafeMRCDeviceCollection::disable_all() {
  for (auto device : get_safe_mrc_devices()) {
    send_command_to_device(device, MRCCmd{MRCMode::FREE, 0.0f});
  }
}

void SafeMRCDeviceCollection::refresh_one(int i) {
  auto device = get_safe_mrc_devices()[i];
  send_command_to_device(device, MRCCmd{MRCMode::DEBUG, 0.0f});
}

void SafeMRCDeviceCollection::refresh_all() {
  for (auto device : get_safe_mrc_devices()) {
    send_command_to_device(device, MRCCmd{MRCMode::REFRESH, 0.0f});
  }
}


void SafeMRCDeviceCollection::set_zero_all() {
  for (auto device : get_safe_mrc_devices()) {
    send_command_to_device(device, MRCCmd{MRCMode::ZERO, 0.0f});
  }
}

void SafeMRCDeviceCollection::mrc_control_one(int i, const MRCCmd& cmd) {
  auto device = get_safe_mrc_devices()[i];
  send_command_to_device(device, cmd);
}

void SafeMRCDeviceCollection::mrc_control_all(const std::vector<MRCCmd>& cmds) {
  auto devices = get_safe_mrc_devices();
  if (cmds.size() != devices.size()) {
    std::cerr << "Error: Command size does not match device size!" << std::endl;
    return;
  }
  for (size_t i = 0; i < cmds.size(); ++i) {
    mrc_control_one(i, cmds[i]);
  }
}

std::vector<MRC> SafeMRCDeviceCollection::get_mrcs() const {
  std::vector<MRC> mrcs;
  for (auto device : get_safe_mrc_devices()) {
    mrcs.push_back(device->get_mrc());
  }
  return mrcs;
}

void SafeMRCDeviceCollection::send_command_to_device(std::shared_ptr<SafeMRCRS485Device> device, 
                                                      const MRCCmd& cmd) { 
  MRCCmdFrame cmd_frame =
    device->create_cmd_frame(static_cast<MRCMode>(cmd.mode), cmd.current_A);
  std::vector<uint8_t> data(sizeof(cmd_frame));
  std::memcpy(data.data(), &cmd_frame, sizeof(cmd_frame));
  rs485_serial_.write_rs485_data(data);
}


std::vector<std::shared_ptr<SafeMRCRS485Device>> SafeMRCDeviceCollection::get_safe_mrc_devices() const {
  std::vector<std::shared_ptr<SafeMRCRS485Device>> safe_mrc_devices;
  for (const auto& [id, device] : device_collection_->get_devices()) {
    auto safe_mrc_device = std::dynamic_pointer_cast<SafeMRCRS485Device>(device);
    if (safe_mrc_device) {
      safe_mrc_devices.push_back(safe_mrc_device);
    }
  }
  return safe_mrc_devices;
}

}  // namespace safe_mrc