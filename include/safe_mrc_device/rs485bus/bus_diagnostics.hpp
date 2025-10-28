/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/
#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace safe_mrc {

class BusDiagnostics {
 public:
  struct DeviceStats {
    uint8_t id;
    uint64_t tx_attempts{0};
    uint64_t tx_successes{0};
    uint64_t rx_attempts{0};
    uint64_t rx_successes{0};
  };

  struct DeviceReport {
    uint8_t id;
    uint64_t tx_attempts;
    uint64_t tx_successes;
    uint64_t rx_attempts;
    uint64_t rx_successes;
    double tx_success_rate;
    double rx_success_rate;
  };

  BusDiagnostics() = default;
  ~BusDiagnostics() = default;

  void register_device(uint8_t id);
  void record_tx_attempt(uint8_t id);
  void record_tx_success(uint8_t id);
  void record_rx_attempt(uint8_t id);
  void record_rx_success(uint8_t id);

  std::vector<DeviceReport> collect_reports() const;
  void show_reports();

 private:
  mutable std::mutex mutex_;
  mutable std::unordered_map<uint8_t, DeviceStats> stats_;
};

}  // namespace safe_mrc

