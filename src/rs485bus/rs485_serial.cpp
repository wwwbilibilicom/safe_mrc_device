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

// （可选调试）打印待发送帧
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

// （可选调试）尝试取一帧并打印
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
// 读取串口 → 推入环形缓冲 → 批量提取完整帧 → 按设备ID路由到对应FIFO
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
// 优先使用“同步快路径”（头已对齐且字节足够），否则进入“搜头模式”。
// CRC 错误时仅丢 1 字节实现快速重同步；找不到头时保留 1 字节尾巴防跨界丢头。
void RS485Serial::extractFramesFromRingBuffer() {
  constexpr uint8_t H0 = 0xFE;
  constexpr uint8_t H1 = 0xEE;
  const size_t FRAME_LEN = sizeof(MRCFdkFrame);

  while (true) {
    // A) 不够头部就退出，等待更多字节
    size_t avail = rx_ring_.size();
    // std::cout << "[RS485 RX] Available bytes: " << avail << std::endl;
    if (avail < 2) {
      // std::cout << "[RS485 RX] Not enough bytes for header, waiting for more data." << std::endl;
      break;
    }

    // B) 同步快路径：ring 起始两字节就是头
    uint8_t b0 = 0, b1 = 0;
    if (rx_ring_.peek_byte(0, b0) && rx_ring_.peek_byte(1, b1) &&
        b0 == H0 && b1 == H1)
    {
      if (avail < FRAME_LEN) {
        // std::cout << "[RS485 RX] Incomplete frame, waiting for more data." << std::endl;
        // 头已对齐但数据不够，等待下一批
        break;
      }

      // 取一帧并校验 CRC（末尾 2 字节 CRC16）
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
        continue;  // 继续榨干下一帧
      } else {
        // CRC 错误：丢 1 字节重同步
        rx_ring_.consume(1);
        continue;
      }
    }

    // C) 搜头模式：从任意位置搜索 H0 H1
    size_t off = rx_ring_.find_header_2B(H0, H1, 0);
    if (off == rx_ring_.size()) {
      // 当前数据中没有头：保留 1 字节尾巴避免跨界丢头
      if (avail > 1) rx_ring_.consume(avail - 1);
      break;
    }

    // 推进到头位置
    if (off > 0) {
      rx_ring_.consume(off);
      avail -= off;
      if (avail < 2) break; // 稳妥保护
    }

    // 此时 0 位置应是头。检查是否有完整一帧
    avail = rx_ring_.size();
    if (avail < FRAME_LEN) break;

    // 取一帧并 CRC 校验
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
      // CRC 错误：左移 1 字节继续同步
      rx_ring_.consume(1);
      continue;
    }
  }
}

// 如果你仍保留 parseOneFrame 接口，提供与上面一致的安全实现；
// 也可仅依赖 extractFramesFromRingBuffer()，把 parseOneFrame 留作内部工具。
bool RS485Serial::parseOneFrame(MRCFdkFrame& frame_out) {
  constexpr uint8_t H0 = 0xFE;
  constexpr uint8_t H1 = 0xEE;
  const size_t FRAME_LEN = sizeof(MRCFdkFrame);

  size_t avail = rx_ring_.size();
  if (avail < 2) return false;

  // 快路径：0/1 正好是头
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

  // 搜头
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
// LIFO 取命令 → 半双工保护（等待 RX/TX 空闲）→ 写串口 → 失败回推栈重试
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
        // 给从机应答时间（视物理层收发延时/转换器特性可调）
        std::this_thread::sleep_for(std::chrono::microseconds(rx_delay_us_));
      }

      if (requeue) {
        // 写入失败或总线忙：重新入栈，保证命令不会丢
        tx_lifo_.push(cmd);
      }

      // 轻微降载
      std::this_thread::sleep_for(std::chrono::microseconds(10));

    } catch (const std::exception& e) {
      set_last_error(e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

}  // namespace safe_mrc
