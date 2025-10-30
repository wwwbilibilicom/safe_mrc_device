/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include <safe_mrc_device/rs485bus/bus_detector.hpp>

#include <iomanip>
#include <limits>
#include <sstream>

namespace safe_mrc {

// ============================================================================
// BusDetector Implementation
// ============================================================================

BusDetector::BusDetector(bool enabled)
    : enabled_(enabled) {}

void BusDetector::recordRequestStart(uint8_t device_id) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Initialize device stats if not exists
  if (device_stats_.find(device_id) == device_stats_.end()) {
    DeviceStats stats;
    stats.device_id = device_id;
    stats.min_response_time_us = std::numeric_limits<double>::max();
    device_stats_[device_id] = stats;
  }

  device_stats_[device_id].total_requests++;
  pending_requests_[device_id] = std::chrono::steady_clock::now();
}

void BusDetector::recordSuccess(uint8_t device_id) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto it = device_stats_.find(device_id);
  if (it == device_stats_.end()) {
    return;  // Device not found, shouldn't happen
  }

  // Calculate response time from pending request
  auto pending_it = pending_requests_.find(device_id);
  if (pending_it == pending_requests_.end()) {
    return;  // No pending request for this device
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                      end_time - pending_it->second)
                      .count();
  double response_time_us = static_cast<double>(duration);

  DeviceStats& stats = it->second;
  stats.successful_responses++;
  stats.last_update = std::chrono::steady_clock::now();

  // Update response time statistics
  updateAvgResponseTime(stats, response_time_us);

  if (response_time_us > stats.max_response_time_us) {
    stats.max_response_time_us = response_time_us;
  }

  if (response_time_us < stats.min_response_time_us) {
    stats.min_response_time_us = response_time_us;
  }

  // Update success rate
  updateSuccessRate(stats);

  // Remove pending request
  pending_requests_.erase(pending_it);
}

void BusDetector::recordFailure(uint8_t device_id) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto it = device_stats_.find(device_id);
  if (it == device_stats_.end()) {
    return;  // Device not found, shouldn't happen
  }

  DeviceStats& stats = it->second;
  stats.failed_responses++;
  stats.last_update = std::chrono::steady_clock::now();

  // Update success rate
  updateSuccessRate(stats);

  // Remove pending request if exists
  pending_requests_.erase(device_id);
}

DeviceStats BusDetector::getDeviceStats(uint8_t device_id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = device_stats_.find(device_id);
  if (it != device_stats_.end()) {
    return it->second;
  }

  return DeviceStats();  // Return empty stats if not found
}

std::map<uint8_t, DeviceStats> BusDetector::getAllDeviceStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return device_stats_;
}

void BusDetector::resetStats(uint8_t device_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (device_id == 0) {
    // Reset all devices
    for (auto& pair : device_stats_) {
      DeviceStats& stats = pair.second;
      uint8_t id = stats.device_id;
      stats = DeviceStats();
      stats.device_id = id;
      stats.min_response_time_us = std::numeric_limits<double>::max();
    }
  } else {
    // Reset specific device
    auto it = device_stats_.find(device_id);
    if (it != device_stats_.end()) {
      uint8_t id = it->second.device_id;
      it->second = DeviceStats();
      it->second.device_id = id;
      it->second.min_response_time_us = std::numeric_limits<double>::max();
    }
  }
}

std::string BusDetector::generateReport() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::ostringstream oss;
  oss << "\n===============================================\n";
  oss << "      RS485 Bus Detection Statistics Report      \n";
  oss << "===============================================\n\n";

  if (device_stats_.empty()) {
    oss << "No device statistics available.\n";
    return oss.str();
  }

  oss << std::left;
  oss << std::setw(8) << "DevID"
      << std::setw(12) << "Total Req"
      << std::setw(12) << "Success"
      << std::setw(12) << "Failed"
      << std::setw(14) << "Success Rate"
      << std::setw(14) << "Avg Time(us)"
      << std::setw(14) << "Max Time(us)"
      << std::setw(14) << "Min Time(us)" << "\n";

  oss << std::string(98, '-') << "\n";

  for (const auto& pair : device_stats_) {
    const DeviceStats& stats = pair.second;

    oss << std::setw(8) << static_cast<int>(stats.device_id)
        << std::setw(12) << stats.total_requests
        << std::setw(12) << stats.successful_responses
        << std::setw(12) << stats.failed_responses
        << std::setw(12) << std::fixed << std::setprecision(2)
        << stats.success_rate << "%"
        << std::setw(14) << std::fixed << std::setprecision(2)
        << stats.avg_response_time_us
        << std::setw(14) << std::fixed << std::setprecision(2)
        << stats.max_response_time_us
        << std::setw(14) << std::fixed << std::setprecision(2);

    if (stats.min_response_time_us == std::numeric_limits<double>::max()) {
      oss << "N/A";
    } else {
      oss << stats.min_response_time_us;
    }

    oss << "\n";
  }

  oss << "===============================================\n";

  return oss.str();
}

void BusDetector::setEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
}

bool BusDetector::isEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void BusDetector::updateSuccessRate(DeviceStats& stats) {
  if (stats.total_requests > 0) {
    stats.success_rate =
        (static_cast<double>(stats.successful_responses) /
         static_cast<double>(stats.total_requests)) *
        100.0;
  } else {
    stats.success_rate = 0.0;
  }
}

void BusDetector::updateAvgResponseTime(DeviceStats& stats,
                                        double new_response_time) {
  // Calculate running average using incremental formula
  // new_avg = old_avg + (new_value - old_avg) / count
  if (stats.successful_responses == 1) {
    // First successful response
    stats.avg_response_time_us = new_response_time;
  } else {
    stats.avg_response_time_us =
        stats.avg_response_time_us +
        (new_response_time - stats.avg_response_time_us) /
            static_cast<double>(stats.successful_responses);
  }
}

}  // namespace safe_mrc
