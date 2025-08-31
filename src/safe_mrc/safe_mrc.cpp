/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/safe_mrc/safe_mrc.hpp"
#include <string>
 namespace safe_mrc {


MRC::MRC(MRCType mrc_type, uint8_t rs485_id)
    : enabled_(false),
      state_q_(0.0),
      state_dq_(0.0),
      state_current_(0.0),
      state_collision_flag_(0),
      mode_(MRCMode::FREE),
      rs485_id_(rs485_id),
      mrc_type_(mrc_type) {}


void MRC::set_enabled(bool enable) {this->enabled_ = enable; }

void MRC::update_state(double q, double dq, double current, uint8_t collision_flag) {
    state_q_ = q;
    state_dq_ = dq;
    state_current_ = current;
    state_collision_flag_ = collision_flag;
}

 }//namespace safe_mrc
