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
#include <vector>
#include <safe_mrc_device/safe_mrc/safe_mrc_protocol.h>
#include <safe_mrc_device/safe_mrc/safe_mrc.hpp>
#include <safe_mrc_device/rs485bus/rs485_device_collection.hpp>

namespace safe_mrc {

class SafeMRCRS485Device: public safe_mrc::RS485Device {
public:
    SafeMRCRS485Device(MRC& mrc);
    ~SafeMRCRS485Device();

    MRC & get_mrc(){ return mrc_; }

    void callback(MRCFdkFrame & frame);

    MRCCmdFrame create_cmd_frame(MRCMode, float current_A);
    void parse_fdk_frame(MRCFdkFrame &frame); //use in rs485 device

private:
    MRC & mrc_;

    MRCMode mrc_mode_;

};

}
