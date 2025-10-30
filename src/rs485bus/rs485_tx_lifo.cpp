/**************************************************
 * RS485 Transmit LIFO (thread-safe) - Implementation
 *
 * - Implements LIFO semantics (last-in-first-out)
 * - When full: reject new frame (return false)
 * - Supports both blocking and non-blocking pop
 * - Used by RS485Serial::txThread() for queued sending
 *
 * Copyright (c) 2025
 * MIT License
 **************************************************/

#include "safe_mrc_device/rs485bus/rs485_tx_lifo.hpp"

namespace safe_mrc {

RS485TxLifo::RS485TxLifo(size_t max_size)
    : max_size_(max_size) {}

void RS485TxLifo::set_max_size(size_t n) {
  std::lock_guard<std::mutex> lk(mtx_);
  max_size_ = n;
}

size_t RS485TxLifo::size() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return s_.size();
}

void RS485TxLifo::clear() {
  std::lock_guard<std::mutex> lk(mtx_);
  s_.clear();
}

bool RS485TxLifo::push(const MRCCmdFrame& f) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (s_.size() >= max_size_) {
      // Reject new frame when queue is full
      ++overflow_reject_new_;
      return false;
    }
    s_.push_back(f);  // Push newest command to top of stack
  }
  cv_.notify_one();
  return true;
}

bool RS485TxLifo::try_pop(MRCCmdFrame& out) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (s_.empty()) return false;
  out = s_.back();
  s_.pop_back();
  return true;
}

bool RS485TxLifo::wait_and_pop(MRCCmdFrame& out, int timeout_ms) {
  std::unique_lock<std::mutex> lk(mtx_);
  if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                    [&] { return !s_.empty(); }))
    return false;
  out = s_.back();
  s_.pop_back();
  return true;
}

uint64_t RS485TxLifo::overflow_reject_new_count() const {
  return overflow_reject_new_.load();
}

}  // namespace safe_mrc
