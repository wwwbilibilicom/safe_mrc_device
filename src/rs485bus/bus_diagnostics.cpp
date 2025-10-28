/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/rs485bus/bus_diagnostics.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>

namespace safe_mrc {

void BusDiagnostics::register_device(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.try_emplace(id, DeviceStats{id});
}

void BusDiagnostics::record_tx_attempt(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = stats_.find(id);
  if (it != stats_.end()) {
    ++it->second.tx_attempts;
  }
}

void BusDiagnostics::record_tx_success(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = stats_.find(id);
  if (it != stats_.end()) {
    ++it->second.tx_successes;
  }
}

void BusDiagnostics::record_rx_attempt(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = stats_.find(id);
  if (it != stats_.end()) {
    ++it->second.rx_attempts;
  }
}

void BusDiagnostics::record_rx_success(uint8_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = stats_.find(id);
  if (it != stats_.end()) {
    ++it->second.rx_successes;
  }
}

std::vector<BusDiagnostics::DeviceReport> BusDiagnostics::collect_reports()
    const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DeviceReport> reports;
  reports.reserve(stats_.size());
  for (const auto& [id, stats] : stats_) {
    auto tx_attempts = stats.tx_attempts;
    auto rx_attempts = stats.rx_attempts;
    double tx_rate = tx_attempts > 0
                         ? static_cast<double>(stats.tx_successes) /
                               static_cast<double>(tx_attempts)
                         : 0.0;
    double rx_rate = rx_attempts > 0
                         ? static_cast<double>(stats.rx_successes) /
                               static_cast<double>(rx_attempts)
                         : 0.0;
    reports.push_back(DeviceReport{id,        tx_attempts, stats.tx_successes,
                                  rx_attempts, stats.rx_successes, tx_rate,
                                  rx_rate});
  }
  std::sort(reports.begin(), reports.end(),
            [](const DeviceReport& lhs, const DeviceReport& rhs) {
              return lhs.id < rhs.id;
            });
  return reports;
}

void BusDiagnostics::show_reports() {

  auto reports = collect_reports();

  if (!reports.empty()) {
    std::cout << "\n[Bus Diagnostics]" << std::endl;
    std::cout << std::setw(15) << "RS485 ID" << std::setw(18)
              << "TX Success (%)" << std::setw(18) << "RX Success (%)"
              << std::setw(18) << "TX Attempts" << std::setw(15)
              << "RX Attempts" << std::endl;
    std::cout << std::string(84, '-') << std::endl;
    for (const auto& report : reports) {
      std::cout << std::setw(15) << static_cast<int>(report.id)
                << std::setw(18) << report.tx_success_rate * 100.0
                << std::setw(18) << report.rx_success_rate * 100.0
                << std::setw(18) << report.tx_attempts << std::setw(15)
                << report.rx_attempts << std::endl;
    }
    std::cout << std::string(84, '-') << std::endl;
  }
  
}

}  // namespace safe_mrc

