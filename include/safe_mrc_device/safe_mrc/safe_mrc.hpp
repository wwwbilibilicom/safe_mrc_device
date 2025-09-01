/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

#include <cstdint>
#include "safe_mrc_device/safe_mrc/safe_mrc_protocol.h"
#include <string>

namespace safe_mrc {

enum class MRCType{
    ROTARY96,
    ROTARY52,
    LINEAR
};

class MRC {

friend class SafeMRCRS485Device;
public:
    MRC(MRCType mrc_type, uint8_t rs485_id);
    ~MRC() = default;

    bool is_enabled() const { return enabled_; }

    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_current() const { return state_current_; }
    uint8_t get_collision_flag() const { return state_collision_flag_; }
    uint8_t get_rs485_id() const { return rs485_id_; }
    MRCType get_mrc_type() const { return mrc_type_; }

    std::string get_mode_string() const {
        switch (mode_) {
            case MRCMode::FREE:
                return "FREE";
            case MRCMode::FIX_LIMIT:
                return "FIX_LIMIT";
            case MRCMode::ADAPTATION:
                return "ADAPTATION";
            case MRCMode::DEBUG:
                return "DEBUG";
            case MRCMode::RESET:
                return "RESET";
            case MRCMode::ZERO:
                return "ZERO";
            default:
                return "UNKNOWN";
        }
    }

    MRCMode get_mode() const {return mode_;}
    void set_enabled(bool enable);
    
private:
    bool enabled_;
    
    double state_q_, state_dq_, state_current_;
    uint8_t state_collision_flag_;

    MRCMode mode_;

    uint8_t rs485_id_;

    MRCType mrc_type_;
    
protected:
    void update_state(double q, double dq, double current, uint8_t collision_flag);
};
}//namespace safe_mrc