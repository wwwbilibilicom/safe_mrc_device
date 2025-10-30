/**************************************************
 * RS485 Receive FIFO (thread-safe)
 * - FIFO semantics (oldest-out)
 * - When full: drop the oldest frame (keep newest)
 * - Supports both blocking and non-blocking pop
 * - Optional pop by device ID (like a simple filter)
 *
 * Copyright (c) 2025
 * MIT License
 **************************************************/
#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <chrono>

#include <safe_mrc_device/safe_mrc/safe_mrc_protocol.h> // MRCFdkFrame

namespace safe_mrc {

/**
 * @brief Thread-safe receive FIFO for RS485 frames.
 *
 * Design goals:
 *  - Preserve newest data under back pressure by dropping the oldest frame when full.
 *  - Provide both blocking and non-blocking APIs.
 *  - Allow optional retrieval by device ID without reordering other frames.
 *
 * Note:
 *  - All methods are thread-safe.
 *  - "wait_and_pop*" methods support timeout; return false on timeout.
 */
class RS485RxFifo {
public:
  /// Construct with a maximum capacity (number of frames).
  explicit RS485RxFifo(size_t max_size = 256);

  /// Change maximum capacity (not thread-hazardous).
  void set_max_size(size_t n);

  /// Current number of frames queued.
  size_t size() const;

  /// Clear all queued frames.
  void clear();

  /// Push a frame (drop the oldest if the queue is full).
  void push(const MRCFdkFrame& f);

  /// Non-blocking pop: FIFO order (oldest first).
  bool try_pop(MRCFdkFrame& out);

  /// Blocking pop: FIFO order (oldest first). Returns false on timeout.
  bool wait_and_pop(MRCFdkFrame& out, int timeout_ms = 100);

  /// Non-blocking pop by device ID (preserve other frames' order).
  bool try_pop_by_id(uint8_t id, MRCFdkFrame& out);

  /// Blocking pop by device ID (preserve other frames' order). Returns false on timeout.
  bool wait_and_pop_by_id(uint8_t id, MRCFdkFrame& out, int timeout_ms = 100);

  /// Number of times the queue was full and the oldest frame was dropped.
  uint64_t overflow_drop_old_count() const;

  void show();

private:
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<MRCFdkFrame> q_;
  size_t max_size_;
  std::atomic<uint64_t> overflow_drop_old_{0};
};

}  // namespace safe_mrc
