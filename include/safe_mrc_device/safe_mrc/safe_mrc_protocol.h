/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __MRC_PROTOCOL_H__
#define __MRC_PROTOCOL_H__


#include <stdint.h>

namespace safe_mrc {

#pragma pack(1)
    typedef struct MRCCmdFrame     //message buffer typedef for the command protocol
    {
        uint8_t head[2];                // 2 bytes 0xFE, 0xEE
        uint8_t id;                     // 1 bytes
        uint8_t mode;                  // 1 bytes
        int32_t des_coil_current;            // 2 bytes
        uint16_t CRC16Data;             // 4 bytes
    } MRCCmdFrame;                 // 10 bytes 
#pragma pack()

#pragma pack(1)
    typedef struct MRCFdkFrame //message buffer typedef for the feedback protocol
    {
        uint8_t head[2];            // 2 bytes
        uint8_t id;                 // 1 bytes
        uint8_t mode;              // 1 bytes
        uint8_t collision_flag;             // 1 bytes 0x00: safely, 0x01: collision happened.
        int32_t encoder_position;     // 4 bytes
        int32_t encoder_velocity;    // 2 bytes
        int16_t present_current;    // 2 bytes
        uint16_t CRC16Data;         // 2 bytes
    } MRCFdkFrame;             // 17 bytes
#pragma pack()


struct StateResult{
    uint8_t id;
    uint8_t mode;
    uint8_t collision;
    float   position;
    float   velocity;
    float   current;
    bool     crc_ok;
    float    frequency_hz;
};

enum class MRCMode{
    FREE,
    FIX_LIMIT,
    ADAPTATION,
    DEBUG,
    RESET,
    ZERO,
    REFRESH
};

struct MRCCmd{
    MRCMode mode;
    float   current_A; // A
};

constexpr uint8_t HEAD0 = 0xFE;
constexpr uint8_t HEAD1 = 0xEE;

} // namespace safe_mrc



#endif /* __MRC_PROTOCOL_H__ */
