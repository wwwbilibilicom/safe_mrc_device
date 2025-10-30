#include "safe_mrc_device/rs485bus/rs485_serial.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <iomanip>

namespace safe_mrc {

RS485Serial::RS485Serial(const std::string& port)
    : serial::Serial(port, 4000000, serial::Timeout::simpleTimeout(20)),
      port_(port),
      rx_ring_(1u << 14),   // 16 KB ring buffer
      tx_lifo_(128) {       // 128-entry TX LIFO
  if (!this->isOpen()) {
    throw RS485SerialException("Failed to open port " + port);
  }
  std::cout << "[RS485] Port opened at 4 Mbps: " << port_ << std::endl;

  flushInput();
  flushOutput();
  initialized_ = true;

  enable_rx_thread();
  enable_tx_thread();
}

RS485Serial::~RS485Serial() {
  disable_rx_thread();
  disable_tx_thread();
  if (isOpen()) {
    close();
    std::cout << "[RS485] Port closed: " << port_ << std::endl;
  }
}

// ----------------------------- Thread Controls ------------------------------

bool RS485Serial::enable_rx_thread() {
  if (rx_thread_running_) return true;
  rx_thread_running_ = true;
  rx_thread_ = std::thread(&RS485Serial::rxThread, this);
  return true;
}

void RS485Serial::disable_rx_thread() {
  if (!rx_thread_running_) return;
  rx_thread_running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
}

bool RS485Serial::enable_tx_thread() {
  if (tx_thread_running_) return true;
  tx_thread_running_ = true;
  tx_thread_ = std::thread(&RS485Serial::txThread, this);
  return true;
}

void RS485Serial::disable_tx_thread() {
  if (!tx_thread_running_) return;
  tx_thread_running_ = false;
  if (tx_thread_.joinable()) tx_thread_.join();
}

// ----------------------------- TX Interface ---------------------------------

// Get or create per-device FIFO (thread-safe)
RS485RxFifo& RS485Serial::getOrCreateFifo(uint8_t device_id) {
  std::lock_guard<std::mutex> lock(rx_fifos_mutex_);
  auto it = rx_fifos_per_device_.find(device_id);
  if (it == rx_fifos_per_device_.end()) {
    // Use try_emplace to construct in-place, avoiding copy
    auto result = rx_fifos_per_device_.try_emplace(device_id, 128);
    std::cout << "[RS485] Created RX FIFO for device ID: "
              << static_cast<int>(device_id) << std::endl;
    return result.first->second;
  }
  return it->second;
}

// Validate device ID is in expected range (0-15)
bool RS485Serial::isValidDeviceId(uint8_t device_id) const {
  return device_id <= 15;
}

// ----------------------------- TX Interface ---------------------------------

bool RS485Serial::pushTxFrame(const MRCCmdFrame& frame) {
  return tx_lifo_.push(frame);
}

// (Optional debug) Print TX frame in hex format
void RS485Serial::printTxFrameHex(MRCCmdFrame &frame)
{
  constexpr size_t len = sizeof(MRCCmdFrame);
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&frame);

  std::ostringstream oss;
  oss << "[TX] ID=" << std::dec << static_cast<int>(frame.id)
      << " Mode=" << static_cast<int>(frame.mode)
      << " CRC=0x" << std::hex << std::uppercase << frame.CRC16Data
      << " DATA=";

  oss << std::hex << std::setfill('0') << std::uppercase;
  for (size_t i = 0; i < len; ++i) {
    oss << std::setw(2) << static_cast<int>(data[i]);
    if (i != len - 1) oss << " ";
  }

  std::cout << oss.str() << std::dec << std::endl;
}

// ----------------------------- RX Interface ---------------------------------

bool RS485Serial::popRxFrame(uint8_t device_id, MRCFdkFrame& out_frame, int timeout_ms) {
  return getOrCreateFifo(device_id).wait_and_pop(out_frame, timeout_ms);
}

// (Optional debug) Try to pop one frame and print it
bool RS485Serial::printRxFrame(uint8_t device_id)
{
  MRCFdkFrame frame;
  if (popRxFrame(device_id, frame, 100)) {
    std::cout << "Received Frame - ID: " << static_cast<int>(frame.id)
              << " Mode: " << static_cast<int>(frame.mode)
              << " Position: " << frame.encoder_position
              << " Velocity: " << frame.encoder_velocity
              << " Current: " << frame.present_current
              << std::endl;
    return true;
  } else {
    std::cout << "No frame received within timeout." << std::endl;
  }
  return false;
}

void RS485Serial::showRxRingBuffer() {
  rx_ring_.show();
}

// ----------------------------- RX Thread ------------------------------------
// Read serial → Push to ring buffer → Extract frames → Route by device ID to corresponding FIFO
void RS485Serial::rxThread() {
  uint8_t temp[2048];
  while (rx_thread_running_) {
    try {
      size_t avail = available();

      if (avail == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        continue;
      }
      // std::cout << "[RS485 RX] Available bytes: " << avail << std::endl;

      if (avail > sizeof(temp)) avail = sizeof(temp);
      rx_busy_.store(true, std::memory_order_relaxed);
      size_t n = read(temp, avail);
      rx_busy_.store(false, std::memory_order_relaxed);

      if (n > 0) {
        rx_ring_.push(temp, n);
        // std::cout << "[RS485 RX] Read " << n << " bytes." << std::endl;
      }

      extractFramesFromRingBuffer();

    } catch (const std::exception& e) {
      set_last_error(e.what());
      rx_busy_.store(false, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

// ----------------------------- Frame Extractor ------------------------------
// Prioritize "sync fast path" (header aligned with sufficient bytes), otherwise enter "header search mode".
// On CRC error, discard only 1 byte for fast resync; if header not found, keep 1 tail byte to prevent cross-boundary header loss.
void RS485Serial::extractFramesFromRingBuffer() {
  constexpr uint8_t H0 = 0xFE;
  constexpr uint8_t H1 = 0xEE;
  const size_t FRAME_LEN = sizeof(MRCFdkFrame);

  while (true) {
    // A) Exit if not enough for header, wait for more bytes
    size_t avail = rx_ring_.size();
    // std::cout << "[RS485 RX] Available bytes: " << avail << std::endl;
    if (avail < 2) {
      // std::cout << "[RS485 RX] Not enough bytes for header, waiting for more data." << std::endl;
      break;
    }

    // B) Sync fast path: first two bytes of ring are header
    uint8_t b0 = 0, b1 = 0;
    if (rx_ring_.peek_byte(0, b0) && rx_ring_.peek_byte(1, b1) &&
        b0 == H0 && b1 == H1)
    {
      if (avail < FRAME_LEN) {
        // std::cout << "[RS485 RX] Incomplete frame, waiting for more data." << std::endl;
        // Header aligned but not enough data, wait for next batch
        break;
      }

      // Extract one frame and validate CRC (last 2 bytes are CRC16)
      uint8_t tmp[sizeof(MRCFdkFrame)];
      rx_ring_.peek(0, tmp, FRAME_LEN);

      uint16_t crc_recv = 0;
      std::memcpy(&crc_recv, tmp + FRAME_LEN - 2, sizeof(uint16_t));
      uint16_t crc_calc = crc_ccitt(0xFFFF, tmp, FRAME_LEN - 2);

      if (crc_recv == crc_calc) {
        MRCFdkFrame f{};
        std::memcpy(&f, tmp, FRAME_LEN);

        // Validate device ID before routing to FIFO
        if (isValidDeviceId(f.id)) {
          getOrCreateFifo(f.id).push(f);
#ifdef RS485_DEBUG_ROUTING
          std::cout << "[RS485] Routed frame to device "
                    << static_cast<int>(f.id) << " (fast path)" << std::endl;
#endif
        } else {
          std::cerr << "[RS485] WARNING: Invalid device ID "
                    << static_cast<int>(f.id) << " (expected 0-15), discarding frame"
                    << std::endl;
        }

        rx_ring_.consume(FRAME_LEN);
        continue;  // Continue extracting next frame
      } else {
        // CRC error: discard 1 byte and resync
        rx_ring_.consume(1);
        continue;
      }
    }

    // C) Header search mode: search for H0 H1 from any position
    size_t off = rx_ring_.find_header_2B(H0, H1, 0);
    if (off == rx_ring_.size()) {
      // No header in current data: keep 1 tail byte to avoid cross-boundary header loss
      if (avail > 1) rx_ring_.consume(avail - 1);
      break;
    }

    // Advance to header position
    if (off > 0) {
      rx_ring_.consume(off);
      avail -= off;
      if (avail < 2) break; // Safety guard
    }

    // Now position 0 should be header. Check if we have a complete frame
    avail = rx_ring_.size();
    if (avail < FRAME_LEN) break;

    // Extract one frame and validate CRC
    uint8_t tmp[sizeof(MRCFdkFrame)];
    rx_ring_.peek(0, tmp, FRAME_LEN);
    uint16_t crc_recv = 0;
    std::memcpy(&crc_recv, tmp + FRAME_LEN - 2, sizeof(uint16_t));
    uint16_t crc_calc = crc_ccitt(0xFFFF, tmp, FRAME_LEN - 2);

    std::cout << "crc_recv: " << crc_recv << " crc_calc: " << crc_calc << std::endl;
    

    if (crc_recv == crc_calc) {
      MRCFdkFrame f{};
      std::memcpy(&f, tmp, FRAME_LEN);

      // Validate device ID before routing to FIFO
      if (isValidDeviceId(f.id)) {
        getOrCreateFifo(f.id).push(f);
#ifdef RS485_DEBUG_ROUTING
        std::cout << "[RS485] Routed frame to device "
                  << static_cast<int>(f.id) << " (search path)" << std::endl;
#endif
      } else {
        std::cerr << "[RS485] WARNING: Invalid device ID "
                  << static_cast<int>(f.id) << " (expected 0-15), discarding frame"
                  << std::endl;
      }

      rx_ring_.consume(FRAME_LEN);
      continue;
    } else {
      // CRC error: shift 1 byte left and continue syncing
      rx_ring_.consume(1);
      continue;
    }
  }
}

// If you still keep the parseOneFrame interface, provide a consistent safe implementation.
// Alternatively, rely solely on extractFramesFromRingBuffer() and keep parseOneFrame as an internal tool.
bool RS485Serial::parseOneFrame(MRCFdkFrame& frame_out) {
  constexpr uint8_t H0 = 0xFE;
  constexpr uint8_t H1 = 0xEE;
  const size_t FRAME_LEN = sizeof(MRCFdkFrame);

  size_t avail = rx_ring_.size();
  if (avail < 2) return false;

  // Fast path: bytes 0/1 are exactly the header
  uint8_t b0 = 0, b1 = 0;
  if (rx_ring_.peek_byte(0, b0) && rx_ring_.peek_byte(1, b1) &&
      b0 == H0 && b1 == H1)
  {
    if (avail < FRAME_LEN) return false;

    uint8_t tmp[sizeof(MRCFdkFrame)];
    rx_ring_.peek(0, tmp, FRAME_LEN);

    uint16_t crc_recv = 0;
    std::memcpy(&crc_recv, tmp + FRAME_LEN - 2, sizeof(uint16_t));
    uint16_t crc_calc = crc_ccitt(0xFFFF, tmp, FRAME_LEN - 2);

    if (crc_recv != crc_calc) {
      rx_ring_.consume(1);
      return false;
    }
    std::memcpy(&frame_out, tmp, FRAME_LEN);
    rx_ring_.consume(FRAME_LEN);
    return true;
  }

  // Header search
  size_t off = rx_ring_.find_header_2B(H0, H1, 0);
  if (off == rx_ring_.size()) {
    if (avail > 1) rx_ring_.consume(avail - 1);
    return false;
  }
  if (off > 0) rx_ring_.consume(off);

  if (rx_ring_.size() < FRAME_LEN) return false;

  uint8_t tmp[sizeof(MRCFdkFrame)];
  rx_ring_.peek(0, tmp, FRAME_LEN);

  uint16_t crc_recv = 0;
  std::memcpy(&crc_recv, tmp + FRAME_LEN - 2, sizeof(uint16_t));
  uint16_t crc_calc = crc_ccitt(0xFFFF, tmp, FRAME_LEN - 2);

  if (crc_recv != crc_calc) {
    rx_ring_.consume(1);
    return false;
  }

  std::memcpy(&frame_out, tmp, FRAME_LEN);
  rx_ring_.consume(FRAME_LEN);
  return true;
}

// ----------------------------- TX Thread ------------------------------------
// Pop command from LIFO → Half-duplex guard (wait for RX/TX idle) → Write to serial → Requeue on failure
void RS485Serial::txThread() {
  while (tx_thread_running_) {
    try {
      MRCCmdFrame cmd{};
      if (!tx_lifo_.wait_and_pop(cmd, 50)) {
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        continue;
      }

      bool requeue = false;
      auto start = std::chrono::steady_clock::now();
      const auto budget = std::chrono::microseconds(200);

      while (rx_busy_.load(std::memory_order_acquire) ||
             tx_busy_.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() - start >= budget) {
          requeue = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(5));
      }

      if (!requeue) {
        tx_busy_.store(true, std::memory_order_release);
        size_t n = write(reinterpret_cast<const uint8_t*>(&cmd),
                         sizeof(MRCCmdFrame));
        tx_busy_.store(false, std::memory_order_release);

        if (n != sizeof(MRCCmdFrame)) {
          set_last_error("TX short write");
          requeue = true;
        }
        // Give slave time to respond (adjustable based on physical layer transceiver delay)
        std::this_thread::sleep_for(std::chrono::microseconds(rx_delay_us_));
      }

      if (requeue) {
        // Write failed or bus busy: requeue to ensure command is not lost
        tx_lifo_.push(cmd);
      }

      // Light load reduction
      std::this_thread::sleep_for(std::chrono::microseconds(10));

    } catch (const std::exception& e) {
      set_last_error(e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

}  // namespace safe_mrc
