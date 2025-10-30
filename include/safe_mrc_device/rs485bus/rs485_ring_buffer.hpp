/**************************************************
 * Byte Ring Buffer for high-throughput RS485 RX pipelines
 *
 * - Lock-free single-producer/single-consumer friendly (external sync if needed)
 * - O(1) push/peek/consume with wrap-around
 * - Drop-oldest policy on overflow (keep newest bytes)
 *
 * Copyright (c) 2025
 * MIT License
 **************************************************/

#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include <cstddef>

namespace safe_mrc {

/**
 * @brief Fixed-size circular buffer for byte streams (optimized for RS485 RX).
 *
 * - Supports fast push(), peek(), and consume() operations.
 * - Overflow automatically discards oldest data ("drop-oldest" policy).
 * - Designed for real-time serial frame extraction at multi-Mbps rates.
 */
class ByteRingBuffer {
 public:
  /// Construct buffer with capacity rounded up to nearest power of two (min 1024 bytes)
  explicit ByteRingBuffer(size_t capacity_pow2 = 1u << 14);

  /// Total capacity in bytes
  size_t capacity() const;

  /// Number of bytes currently stored
  size_t size() const;

  /// Free space left in buffer
  size_t free_space() const;

  /// Clear all contents
  void clear();

  /// Push new bytes (drops oldest if overflow). Returns number of bytes written.
  size_t push(const uint8_t* data, size_t n);

  /// Copy out up to `n` bytes starting from offset (without consuming)
  size_t peek(size_t offset, uint8_t* out, size_t n) const;

  /// Peek a single byte at logical index (tail + offset)
  bool peek_byte(size_t offset, uint8_t& b) const;

  /// Consume (remove) n bytes from buffer front
  void consume(size_t n);

  /// Find a 2-byte header (e.g. 0xFE 0xEE) starting from offset
  /// Returns offset if found, or size() if not found
  size_t find_header_2B(uint8_t h0, uint8_t h1, size_t start_offset = 0) const;

  /// Number of dropped bytes due to overflow
  uint64_t dropped_bytes() const;

  void show() const;

 private:
  static size_t round_up_pow2(size_t x);

  size_t cap_;
  size_t mask_;
  std::vector<uint8_t> buf_;

  // Logical indexes (monotonic increasing)
  size_t head_;
  size_t tail_;

  std::atomic<uint64_t> dropped_bytes_;
};

}  // namespace safe_mrc
