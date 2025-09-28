/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/rs485bus/rs485_serial.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

namespace safe_mrc {
RS485Serial::RS485Serial(std::string port)
    : serial::Serial(port, 4000000, serial::Timeout::simpleTimeout(20)),
      port_(port),
      initialized_(false),
      rx_delay_us_(200) {
  // this->open();
  if (!this->isOpen()) {
    throw RS485SerialException("Failed to open RS485 serial port: " + port_);
  } else {
    std::cout << "RS485 serial: port opened successfully with " << getBaudrate()
              << " baudrate and" << port_ << std::endl;
  }
  rx_buffer_.clear();
  this->flushInput();
  this->flushOutput();
  initialized_ = true;
}

RS485Serial::~RS485Serial() {
  if (this->isOpen()) {
    this->close();
    std::cout << "RS485 serial: port closed: " << port_ << std::endl;
  }
}

size_t RS485Serial::write_rs485_data(const uint8_t* data) {
  auto start = std::chrono::steady_clock::now();
  auto budget = std::chrono::microseconds(200);
  while (rx_busy_.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() - start >= budget) {
      last_error_ = "rx busy: await rx timeout";
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(5));
  }
  size_t len = write(data, sizeof(MRCCmdFrame));
  std::this_thread::sleep_for(
      std::chrono::microseconds(rx_delay_us_));  // no fifo logical
  return len;
}

void RS485Serial::printf_frame(const uint8_t* data) {
  size_t len = sizeof(MRCCmdFrame);
  for (size_t i = 0; i < len; i++) {
    std::cout << std::hex << (int)data[i] << " ";
  }
  std::cout << std::dec << std::endl;
}

}  // namespace safe_mrc
