/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/
#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <thread>

#include "bus_detector.hpp"
#include "rs485_serial.hpp"

namespace safe_mrc {

class RS485Device {
 public:
  RS485Device(uint8_t rs485_id) : rs485_id_(rs485_id) {}
  ~RS485Device();

  virtual void callback(MRCFdkFrame& frame) = 0;

  uint8_t get_rs485_id() const { return rs485_id_; }

 protected:
  uint8_t rs485_id_;
};

class RS485DeviceCollection {
 public:
  RS485DeviceCollection(RS485Serial& rs485_serial);
  virtual ~RS485DeviceCollection();

  void add_device(const std::shared_ptr<RS485Device>& device);
  void remove_device(const std::shared_ptr<RS485Device>& device);

  void dispatch_frame_callback(safe_mrc::MRCFdkFrame& frame);

  const std::map<uint8_t, std::shared_ptr<RS485Device>>& get_devices() const {
    return devices_;
  }
  RS485Serial& get_rs485_serial() const { return rs485_serial_; }

  // Bus detection interface
  BusDetector & get_bus_detector() { return bus_detector_; }
  const BusDetector & get_bus_detector() const { return bus_detector_; }
  void enable_bus_detection(bool enable) { bus_detector_.setEnabled(enable); }

 private:
  RS485Serial& rs485_serial_;
  std::map<uint8_t, std::shared_ptr<RS485Device>> devices_;
  BusDetector bus_detector_;  // Bus detection and statistics


};
}  // namespace safe_mrc
