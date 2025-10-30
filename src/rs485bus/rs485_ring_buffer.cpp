/**************************************************
 * Byte Ring Buffer - Implementation
 *
 * (Separated from header for clean build structure)
 **************************************************/

#include "safe_mrc_device/rs485bus/rs485_ring_buffer.hpp"
#include <cstring>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace safe_mrc {

// ---- Constructor ----
ByteRingBuffer::ByteRingBuffer(size_t capacity_pow2)
    : cap_(round_up_pow2(capacity_pow2)),
      mask_(cap_ - 1),
      buf_(cap_, 0),
      head_(0),
      tail_(0),
      dropped_bytes_(0) {}

// ---- Helpers ----
size_t ByteRingBuffer::round_up_pow2(size_t x) {
  size_t v = std::max<size_t>(x, 1024);
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
#if SIZE_MAX > 0xffffffff
  v |= v >> 32;
#endif
  v++;
  return v;
}

// ---- Basic info ----
size_t ByteRingBuffer::capacity() const { return cap_; }
size_t ByteRingBuffer::size() const { return head_ - tail_; }
size_t ByteRingBuffer::free_space() const { return cap_ - size(); }
uint64_t ByteRingBuffer::dropped_bytes() const { return dropped_bytes_.load(); }

void ByteRingBuffer::clear() { head_ = tail_ = 0; }

// ---- Push ----
size_t ByteRingBuffer::push(const uint8_t* data, size_t n) {
    // std::cout << "[ByteRingBuffer] Pushing " << n << " bytes" << std::endl;
    // std::cout << "[ByteRingBuffer] Current size: " << size() << ", free space: " << free_space() << std::endl;
  if (n > free_space()) {
    size_t need = n - free_space();
    tail_ += need;
    dropped_bytes_ += need;
  }
  const size_t h = head_ & mask_;
  size_t first = std::min(n, cap_ - h);
  std::memcpy(&buf_[h], data, first);
  if (n > first) {
    std::memcpy(&buf_[0], data + first, n - first);
  }
  head_ += n;
//   std::cout << "[ByteRingBuffer] Pushed " << n << " bytes, new size: " << size() << std::endl;
  return n;
}

// ---- Peek ----
size_t ByteRingBuffer::peek(size_t offset, uint8_t* out, size_t n) const {
  size_t avail = size();
  if (offset >= avail) return 0;
  size_t can = std::min(n, avail - offset);
  size_t start = (tail_ + offset) & mask_;
  size_t first = std::min(can, cap_ - start);
  std::memcpy(out, &buf_[start], first);
  if (can > first) {
    std::memcpy(out + first, &buf_[0], can - first);
  }
  return can;
}

// ---- Peek single byte ----
bool ByteRingBuffer::peek_byte(size_t offset, uint8_t& b) const {
  size_t avail = size();
  if (offset >= avail) return false;
  size_t idx = (tail_ + offset) & mask_;
  b = buf_[idx];
  return true;
}

// ---- Consume ----
void ByteRingBuffer::consume(size_t n) { tail_ += n; }

// ---- Find header ----
size_t ByteRingBuffer::find_header_2B(uint8_t h0, uint8_t h1, size_t start_offset) const {
  size_t avail = size();
  if (start_offset + 2 > avail) return avail;
  for (size_t i = start_offset; i + 1 < avail; ++i) {
    uint8_t b0, b1;
    peek_byte(i, b0);
    peek_byte(i + 1, b1);
    if (b0 == h0 && b1 == h1) {
      return i;
    }
  }
  return avail;
}

void ByteRingBuffer::show() const {
  std::cout << "ByteRingBuffer contents (size=" << size() << "): ";
  for (size_t i = 0; i < size(); ++i) {
    uint8_t b;
    peek_byte(i, b);
    std::cout << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(b) << " ";
  }
  std::cout << std::dec << std::endl;
}

}  // namespace safe_mrc
