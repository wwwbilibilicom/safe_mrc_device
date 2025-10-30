/**************************************************
 * RS485 Receive FIFO (thread-safe) - Implementation
 * 
 * - Implements FIFO semantics (oldest-out)
 * - When full: drop the oldest frame
 * - Supports both blocking and non-blocking pop
 * - Optional pop by device ID (simple filtering)
 *
 * Copyright (c) 2025
 * MIT License
 **************************************************/

#include "safe_mrc_device/rs485bus/rs485_rx_fifo.hpp"

namespace safe_mrc {

RS485RxFifo::RS485RxFifo(size_t max_size)
    : max_size_(max_size) {}

void RS485RxFifo::set_max_size(size_t n) {
  std::lock_guard<std::mutex> lk(mtx_);
  max_size_ = n;
}

size_t RS485RxFifo::size() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return q_.size();
}

void RS485RxFifo::clear() {
  std::lock_guard<std::mutex> lk(mtx_);
  q_.clear();
}

void RS485RxFifo::push(const MRCFdkFrame& f) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (q_.size() >= max_size_) {
      // Drop the oldest frame to keep the most recent one
      q_.pop_front();
      ++overflow_drop_old_;
    }
    q_.push_back(f);
  }
  cv_.notify_one();
}

bool RS485RxFifo::try_pop(MRCFdkFrame& out) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (q_.empty()) return false;
  out = q_.front();
  q_.pop_front();
  return true;
}

bool RS485RxFifo::wait_and_pop(MRCFdkFrame& out, int timeout_ms) {
  std::unique_lock<std::mutex> lk(mtx_);
  if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                    [&] { return !q_.empty(); }))
    return false;
  out = q_.front();
  q_.pop_front();
  return true;
}

bool RS485RxFifo::try_pop_by_id(uint8_t id, MRCFdkFrame& out) {
  std::lock_guard<std::mutex> lk(mtx_);
  for (auto it = q_.begin(); it != q_.end(); ++it) {
    if (it->id == id) {
      out = *it;
      q_.erase(it);
      return true;
    }
  }
  return false;
}

bool RS485RxFifo::wait_and_pop_by_id(uint8_t id, MRCFdkFrame& out,
                                     int timeout_ms) {
  std::unique_lock<std::mutex> lk(mtx_);
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (true) {
    // Try to find a frame for this ID
    for (auto it = q_.begin(); it != q_.end(); ++it) {
      if (it->id == id) {
        out = *it;
        q_.erase(it);
        return true;
      }
    }
    // Wait for new data or timeout
    if (cv_.wait_until(lk, deadline, [&] { return !q_.empty(); }) == false) {
      return false;  // timeout
    }
  }
}

uint64_t RS485RxFifo::overflow_drop_old_count() const {
  return overflow_drop_old_.load();
}


}  // namespace safe_mrc
