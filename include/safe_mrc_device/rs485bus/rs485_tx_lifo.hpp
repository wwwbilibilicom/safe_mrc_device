/**************************************************
 * RS485 Transmit LIFO (thread-safe)
 * - LIFO semantics (last-in-first-out)
 * - When full: reject new frame (return false)
 * - Supports both blocking and non-blocking pop
 * - Intended for TX queue (latest command priority)
 *
 * Copyright (c) 2025
 * MIT License
 **************************************************/
#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

#include <safe_mrc_device/safe_mrc/safe_mrc_protocol.h>  // MRCCmdFrame

namespace safe_mrc {

/**
 * @brief Thread-safe transmit LIFO for RS485 command frames.
 *
 * Design goals:
 *  - Prioritize the most recent command (LIFO semantics).
 *  - Avoid overwriting commands: reject new pushes if full.
 *  - Provide both blocking and non-blocking APIs.
 *
 * Typical usage:
 *  - RS485Serial::txThread() continuously pops from this LIFO
 *    to send the most recent command to the device.
 */
class RS485TxLifo {
public:
  /// Construct with a maximum capacity (number of frames).
  explicit RS485TxLifo(size_t max_size = 256);

  /// Change maximum capacity (thread-safe).
  void set_max_size(size_t n);

  /// Current number of queued command frames.
  size_t size() const;

  /// Clear all queued command frames.
  void clear();

  /// Push a command frame (return false if full).
  bool push(const MRCCmdFrame& f);

  /// Non-blocking pop (LIFO order).
  bool try_pop(MRCCmdFrame& out);

  /// Blocking pop (LIFO order). Returns false on timeout.
  bool wait_and_pop(MRCCmdFrame& out, int timeout_ms = 100);

  /// Number of times push() failed due to full queue.
  uint64_t overflow_reject_new_count() const;

private:
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::vector<MRCCmdFrame> s_;
  size_t max_size_;
  std::atomic<uint64_t> overflow_reject_new_{0};
};

}  // namespace safe_mrc
