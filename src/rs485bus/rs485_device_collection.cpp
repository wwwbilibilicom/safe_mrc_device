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

#include "safe_mrc_device/rs485bus/crc_ccitt.h"

namespace safe_mrc {
RS485Device::~RS485Device(){}

RS485DeviceCollection::RS485DeviceCollection(RS485Serial& rs485_serial)
    : rs485_serial_(rs485_serial) {
}

RS485DeviceCollection::~RS485DeviceCollection() { disable_rx_thread(); }

bool RS485DeviceCollection::enable_rx_thread() {
  if (rs485_serial_.is_initialized() == false) {
    throw std::runtime_error("RS485 serial port is not initialized");
    return false;
  }
  if (rx_thread_running_) return true;  // Already running
  try {
    rx_thread_running_ = true;
    rx_thread_ = std::thread(&RS485DeviceCollection::rxThread, this);
    return true;
  } catch (const std::exception& e) {
    rx_thread_running_ = false;
    rs485_serial_.set_last_error(e.what());
    return false;
  }
}

void RS485DeviceCollection::disable_rx_thread() {
  if (!rx_thread_running_) return;  // Not running

  rx_thread_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

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

void RS485DeviceCollection::rxThread() {
  std::vector<uint8_t> temp(256);
  while (rx_thread_running_) {
    try {
      size_t avail = rs485_serial_.available();
      if (avail == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        continue;
      }
      if (avail > temp.size()) avail = temp.size();
      rs485_serial_.rx_busy_.store(true, std::memory_order_relaxed);
      size_t n = rs485_serial_.read(temp.data(), avail);
      rs485_serial_.rx_busy_.store(false, std::memory_order_relaxed);
      if (n > 0) {
        rs485_serial_.rx_buffer_.insert(rs485_serial_.rx_buffer_.end(),
                                        temp.begin(), temp.begin() + n);
        unpackStream(rs485_serial_.rx_buffer_);
      }
    } catch (const std::exception& e) {
      rs485_serial_.set_last_error(e.what());
      rs485_serial_.rx_busy_.store(false, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

// only can be used for safe mrc frame
bool RS485DeviceCollection::unpackStream(std::vector<uint8_t>& buf) {
  size_t idx = 0, L = buf.size();
  while (idx + sizeof(MRCFdkFrame) <= L) {
    if (buf[idx] != 0xFE || buf[idx + 1] != 0xEE) {
      idx++;
      continue;
    }
    const uint8_t* fr = &buf[idx];
    MRCFdkFrame frame;
    memcpy(&frame, fr, sizeof(MRCFdkFrame));
    uint16_t crc_recv = frame.CRC16Data;
    uint16_t crc_calc = crc_ccitt(0xFFFF, fr, sizeof(MRCFdkFrame) - 2);
    if (crc_recv != crc_calc) {
      idx++;
      continue;
    }  // 同步
    dispatch_frame_callback(frame);
    // 剩余尾巴
    buf.erase(buf.begin(), buf.begin() + idx + sizeof(MRCFdkFrame));
    return true;
  }
  // 保留尾巴
  if (idx > 0) buf.erase(buf.begin(), buf.begin() + idx);
  return false;
}

}  // namespace safe_mrc
