/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include <iostream>
#include <cmath>
#include <safe_mrc_device/rs485bus/crc_ccitt.h>
#include <safe_mrc_device/safe_mrc/safe_mrc_device.hpp>

namespace safe_mrc {

SafeMRCRS485Device::SafeMRCRS485Device(MRC& mrc)
    : RS485Device(mrc.get_rs485_id()),
      mrc_(mrc),
      mrc_mode_(MRCMode::FREE) {}
MRCCmdFrame SafeMRCRS485Device::create_cmd_frame(MRCMode mode, float current_A)
{
    MRCCmdFrame frame;
    frame.head[0] = 0xFE;
    frame.head[1] = 0xEE;
    frame.id = mrc_.get_rs485_id();
    frame.mode = static_cast<uint8_t>(mode);
    int32_t current = static_cast<int32_t>(std::round(current_A * 1000.0f));
    frame.des_coil_current = current;
    frame.CRC16Data = crc_ccitt(0xFFFF, reinterpret_cast<uint8_t*>(&frame), sizeof(frame)-2 );

    return frame;
}
void SafeMRCRS485Device::parse_fdk_frame(MRCFdkFrame &frame)
{
    StateResult result;
    if(frame.id != rs485_id_){
        std::cerr << "Warning: MRC ID mismatch! Expected " << static_cast<int>(rs485_id_) 
                  << ", got " << static_cast<int>(frame.id) << std::endl;
        return;
    }
    uint16_t crc_calculated = crc_ccitt(0xFFFF, reinterpret_cast<uint8_t*>(&frame), sizeof(frame)-2 );
    if(crc_calculated != frame.CRC16Data){
        std::cerr << "Warning: CRC mismatch! Calculated " << std::hex << crc_calculated 
                  << ", received " << std::hex << frame.CRC16Data << std::dec << std::endl;
        return;
    }

    result.position = static_cast<double>(frame.encoder_position)/65535.0; 
    result.velocity = static_cast<double>(frame.encoder_velocity) / 1000.0;
    result.current = static_cast<double>(frame.present_current) / 1000.0;
    result.collision = frame.collision_flag;

    mrc_.update_state(result.position, result.velocity, result.current, result.collision);
}


void SafeMRCRS485Device::callback(MRCFdkFrame & frame){this->parse_fdk_frame(frame);}

}

