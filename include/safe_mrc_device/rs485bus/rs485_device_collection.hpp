/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/
#pragma once

#include <cstdint>
#include "rs485_serial.hpp"
#include <memory>
#include <map>
#include <thread>
#include <atomic>
#include <safe_mrc_device/safe_mrc/safe_mrc_protocol.h>

namespace safe_mrc {

class RS485Device {
public:
    RS485Device(uint8_t rs485_id) : rs485_id_(rs485_id) {}
    virtual ~RS485Device() = default;

    virtual void callback(MRCFdkFrame & frame) = 0;

    uint8_t get_rs485_id() const { return rs485_id_; }
protected:
    uint8_t rs485_id_;
};

class RS485DeviceColllection{
public:
    RS485DeviceColllection(RS485Serial& rs485_serial);
    virtual ~RS485DeviceColllection() = default;

    bool enable_rx_thread();
    void disable_rx_thread();

    void add_device(const std::shared_ptr<RS485Device>& device);
    void remove_device(const std::shared_ptr<RS485Device>& device);
    void dispatch_frame_callback(safe_mrc::MRCFdkFrame & frame);

    const std::map<uint8_t, std::shared_ptr<RS485Device>>& get_devices() const { return devices_; }
    RS485Serial& get_rs485_serial() const { return rs485_serial_; }
private:
    RS485Serial& rs485_serial_;
    std::map<uint8_t, std::shared_ptr<RS485Device>> devices_;

    void rxThread();

    bool rx_thread_running_ = false;
    std::thread rx_thread_;
    bool unpackStream(std::vector<uint8_t>& buf);
};
}



