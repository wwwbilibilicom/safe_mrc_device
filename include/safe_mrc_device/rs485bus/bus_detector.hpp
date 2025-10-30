/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <atomic>

namespace safe_mrc {

/**
 * @brief Device communication statistics structure
 *
 * Records communication metrics for a single device on the RS485 bus
 */
struct DeviceStats {
  uint8_t device_id;                     // Device ID (1-255)
  uint64_t total_requests;               // Total number of requests sent
  uint64_t successful_responses;         // Number of successful responses
  uint64_t failed_responses;             // Number of failed responses (timeout/CRC errors)
  double success_rate;                   // Success rate in percentage (0-100)
  double avg_response_time_us;           // Average response time in microseconds
  double max_response_time_us;           // Maximum response time in microseconds
  double min_response_time_us;           // Minimum response time in microseconds
  std::chrono::steady_clock::time_point last_update;  // Last statistics update time

  DeviceStats()
      : device_id(0),
        total_requests(0),
        successful_responses(0),
        failed_responses(0),
        success_rate(0.0),
        avg_response_time_us(0.0),
        max_response_time_us(0.0),
        min_response_time_us(0.0) {}
};

/**
 * @brief Bus detector for monitoring RS485 device communication
 *
 * This class provides real-time statistics collection for RS485 bus communication,
 * including success rates and response times for each device.
 *
 * Thread-safe design allows concurrent access from multiple threads.
 */
class BusDetector {
 public:
  /**
   * @brief Constructor
   * @param enabled Initial state of statistics collection
   */
  explicit BusDetector(bool enabled = true);

  /**
   * @brief Destructor
   */
  ~BusDetector() = default;

  /**
   * @brief Record the start of a communication request
   * @param device_id Target device ID
   */
  void recordRequestStart(uint8_t device_id);

  /**
   * @brief Record a successful response (auto-calculates response time from last request)
   * @param device_id Device ID
   */
  void recordSuccess(uint8_t device_id);

  /**
   * @brief Record a failed response
   * @param device_id Device ID
   */
  void recordFailure(uint8_t device_id);

  /**
   * @brief Get statistics for a specific device
   * @param device_id Device ID to query
   * @return Device statistics, or empty stats if device not found
   */
  DeviceStats getDeviceStats(uint8_t device_id) const;

  /**
   * @brief Get statistics for all monitored devices
   * @return Map of device ID to statistics
   */
  std::map<uint8_t, DeviceStats> getAllDeviceStats() const;

  /**
   * @brief Reset statistics for a specific device or all devices
   * @param device_id Device ID to reset, or 0 to reset all
   */
  void resetStats(uint8_t device_id = 0);

  /**
   * @brief Generate a human-readable statistics report
   * @return Formatted report string
   */
  std::string generateReport() const;

  /**
   * @brief Enable or disable statistics collection
   * @param enabled True to enable, false to disable
   */
  void setEnabled(bool enabled);

  /**
   * @brief Check if statistics collection is enabled
   * @return True if enabled, false otherwise
   */
  bool isEnabled() const;

  
  void registerDetectorDevice(uint8_t device_id);
  void unregisterDetectorDevice(uint8_t device_id);
  
  private:
  mutable std::mutex mutex_;                        // Mutex for thread-safe access
  std::map<uint8_t, DeviceStats> device_stats_;    // Device statistics storage
  std::map<uint8_t, std::chrono::steady_clock::time_point> pending_requests_;  // Pending request timestamps
  std::atomic<bool> enabled_;                                    // Enable/disable flag
  void registerDeviceStats(uint8_t device_id);
  void unregisterDeviceStats(uint8_t device_id);
  void registerDevicePendingRequest(uint8_t device_id);
  void unregisterDevicePendingRequest(uint8_t device_id);

  /**
   * @brief Update success rate for a device
   * @param stats Device statistics to update
   */
  void updateSuccessRate(DeviceStats& stats);

  /**
   * @brief Update average response time for a device
   * @param stats Device statistics to update
   * @param new_response_time New response time to incorporate
   */
  void updateAvgResponseTime(DeviceStats& stats, double new_response_time);
};

}  // namespace safe_mrc
