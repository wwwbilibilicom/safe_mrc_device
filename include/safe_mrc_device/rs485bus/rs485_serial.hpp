#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <serial/serial.h>
#include "safe_mrc_device/safe_mrc/safe_mrc_protocol.h"
#include "safe_mrc_device/rs485bus/crc_ccitt.h"
#include "safe_mrc_device/rs485bus/rs485_rx_fifo.hpp"
#include "safe_mrc_device/rs485bus/rs485_tx_lifo.hpp"
#include "safe_mrc_device/rs485bus/rs485_ring_buffer.hpp"

namespace safe_mrc {

/**
 * @brief Exception type for RS485 communication errors
 */
class RS485SerialException : public std::runtime_error {
 public:
  explicit RS485SerialException(const std::string& msg)
      : std::runtime_error("RS485Serial error: " + msg) {}
};

/**
 * @brief High-speed RS485 Serial Manager (Master side)
 *
 * This class encapsulates both RX and TX threads for full-duplex coordination
 * at 4 Mbps.  It integrates a ring buffer, FIFO/LIFO queues, and CRC checks.
 */
class RS485Serial : public serial::Serial {
 public:
  explicit RS485Serial(const std::string& port);
  ~RS485Serial();

  // Connection / state
  bool is_initialized() const { return initialized_; }
  const std::string& get_port() const { return port_; }

  // Thread controls
  bool enable_rx_thread();
  void disable_rx_thread();
  bool enable_tx_thread();
  void disable_tx_thread();

  // API for upper layers
  bool pushTxFrame(const MRCCmdFrame& frame);
  bool printRxFrame();
  bool popRxFrame(MRCFdkFrame& out_frame, int timeout_ms = 0);
  void printTxFrameHex(MRCCmdFrame &frame);

  void showRxRingBuffer();

  // Parameters
  void set_rx_delay_us(int us) { rx_delay_us_ = us; }
  void setTimeout(int timeout_us) { timeout_us_ = timeout_us; }

  // Error
  std::string lastError() const { return last_error_; }
  void set_last_error(const std::string& msg) { last_error_ = msg; }


 private:
  // Threads
  void rxThread();
  void txThread();

  // Internal functions
  void extractFramesFromRingBuffer();
  bool parseOneFrame(MRCFdkFrame& frame_out);

 private:
  // Device state
  std::string port_;
  bool initialized_{false};
  std::string last_error_;

  // Timing
  int timeout_us_{20};
  int rx_delay_us_{200};

  // RX structures
  ByteRingBuffer rx_ring_;
  RS485RxFifo    rx_fifo_;
  std::atomic<bool> rx_busy_{false};
  std::thread rx_thread_;
  bool rx_thread_running_{false};

  // TX structures
  RS485TxLifo tx_lifo_;
  std::atomic<bool> tx_busy_{false};
  std::thread tx_thread_;
  bool tx_thread_running_{false};

  // Synchronization
  std::mutex err_mtx_;
};

}  // namespace safe_mrc
