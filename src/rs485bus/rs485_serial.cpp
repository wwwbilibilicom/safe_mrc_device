/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

 #include "safe_mrc_device/rs485bus/rs485_serial.hpp"
 #include <iostream>
 #include <chrono>
 #include <thread>
 #include <mutex>

namespace safe_mrc{
RS485Serial::RS485Serial(std::string port)
    : serial::Serial(port, 4000000, serial::Timeout::simpleTimeout(20)),
      port_(port),
      initialized_(false) {
        this->open();
        if(!this->isOpen()){
            throw RS485SerialException("Failed to open RS485 serial port: " + port_);
        }else{
            std::cout << "RS485 serial: port opened successfully: " << port_ << std::endl;
        }
        rx_buffer_.clear();
        this->flushInput();
        this->flushOutput();
        initialized_ = true;
}


RS485Serial::~RS485Serial(){
    if(this->isOpen()){
        this->close();
        std::cout << "RS485 serial: port closed: " << port_ << std::endl;
    }
}


size_t RS485Serial::write_rs485_data(const std::vector<uint8_t>& data)
{
    auto start = std::chrono::steady_clock::now();
    auto budget = std::chrono::microseconds(200);
    while (rx_busy_.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() - start >= budget) {
            last_error_ = "rx busy: await rx timeout";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    return write(data);
}

}
