/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/rs485bus/rs485_device_collection.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>


namespace safe_mrc {
RS485Device::~RS485Device(){}

RS485DeviceCollection::RS485DeviceCollection(RS485Serial& rs485_serial)
    : rs485_serial_(rs485_serial) {
}

RS485DeviceCollection::~RS485DeviceCollection() {}



void RS485DeviceCollection::add_device(
    const std::shared_ptr<RS485Device>& device) {
  if (!device) return;

  // Add device to our collection
  uint8_t device_id = device->get_rs485_id();
  devices_[device_id] = device;

}

void RS485DeviceCollection::remove_device(
    const std::shared_ptr<RS485Device>& device) {
  if (!device) return;

  uint8_t device_id = device->get_rs485_id();
  auto it = devices_.find(device_id);
  if (it != devices_.end()) {
    // Remove from our collection
    devices_.erase(it);
  }
}

void RS485DeviceCollection::dispatch_frame_callback(
    safe_mrc::MRCFdkFrame& frame) {
  auto it = devices_.find(frame.id);
  if (it != devices_.end()) {
    it->second->callback(frame);
  }
  // Note: Silently ignore frames for unknown devices (this is normal in RS485
  // networks)
}

}  // namespace safe_mrc
