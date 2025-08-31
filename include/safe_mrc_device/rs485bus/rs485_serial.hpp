/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

#include <serial/serial.h>
#include <string>
#include <safe_mrc_device/safe_mrc/safe_mrc_protocol.h>
#include <atomic>


namespace safe_mrc{

// Exception classes for socket operations
class RS485SerialException : public std::runtime_error {
public:
    explicit RS485SerialException(const std::string& message)
        : std::runtime_error("RS485 serial error: " + message) {}
};

class RS485Serial: public serial::Serial {
friend class RS485DeviceColllection;
public:
    RS485Serial(std::string port);
    ~RS485Serial();
    const std::string& get_port() const { return port_; }
    bool is_initialized() const { return initialized_; }

    size_t write_rs485_data(const std::vector<uint8_t>& data);

    void set_last_error(const std::string& error_message) { last_error_ = error_message; }
    // size_t is_data_available(int timeout_us);
    std::string lastError() const { return last_error_; }

    void setTimeout(int timeout_us) { timeout_us_ = timeout_us; }
private:
    std::string port_;
    bool initialized_;
    
    std::string last_error_;
    int timeout_us_{20}; // Default timeout for read operations in microseconds
    
protected:
    std::vector<uint8_t> rx_buffer_;
    std::atomic<bool> rx_busy_{false}; // Indicates if RX thread is processing data


};


}
