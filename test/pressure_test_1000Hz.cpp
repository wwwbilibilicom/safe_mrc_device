//
// SafeMRC 1000 Hz Pressure Test
//

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <safe_mrc_device/safe_mrc/safe_mrc.hpp>
#include <safe_mrc_device/safe_mrc/safeguarder.hpp>

namespace {
std::atomic<bool> running(true);

void handle_sigint(int) {
  running.store(false, std::memory_order_release);
}
}  // namespace

int main() {
  std::string port = "/dev/ttyUSB0";

  std::signal(SIGINT, handle_sigint);

  try {
    std::cout << "=== SafeMRC 1000Hz Pressure Test ===" << std::endl;
    std::cout << "Port: " << port << std::endl;

    // Initialize Safeguarder with RS485 port
    safe_mrc::Safeguarder safeguarder(port);

    // Initialize 4 ROTARY52 MRC devices with IDs 0x01-0x04
    std::vector<safe_mrc::MRCType> mrc_types = {
        safe_mrc::MRCType::ROTARY52, safe_mrc::MRCType::ROTARY52,
        safe_mrc::MRCType::ROTARY52, safe_mrc::MRCType::ROTARY52};
    std::vector<uint8_t> rs485_ids = {0x01, 0x02, 0x03, 0x04};

    safeguarder.init_mrcs(mrc_types, rs485_ids);

    // Enable all and set conservative rx timeout for slow ops
    safeguarder.enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    safeguarder.set_rx_timeout_us(2000);

    // Maintain a safe command (FIX_LIMIT 0.0f) for all devices
    safeguarder.get_mrc_component().mrc_control_all(
        {safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f}});

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    size_t ticks = 0;

      safeguarder.set_rx_delay_us(150);
    // Print format config
    std::cout << std::fixed << std::setprecision(2) << std::left;

    while (running.load(std::memory_order_acquire)) {
      next += std::chrono::milliseconds(1);  // 1 kHz period

      // Refresh all device states
      safeguarder.refresh_all();

      // Once per second, print a table of all device members
      if (++ticks % 1000 == 0) {
        std::cout << "\n[1s Snapshot] ticks=" << ticks << std::endl;
        std::cout << std::setw(15) << "MRC RS485 ID" << std::setw(15)
                  << "Position" << std::setw(15) << "Velocity" << std::setw(15)
                  << "Current" << std::setw(18) << "Collision Flag"
                  << std::setw(10) << "Mode" << std::endl;
        std::cout << std::string(88, '-') << std::endl;

        for (const auto& mrc :
             safeguarder.get_mrc_component().get_mrcs()) {
          std::cout << std::setw(15) << static_cast<int>(mrc.get_rs485_id())
                    << std::setw(15) << mrc.get_position() << std::setw(15)
                    << mrc.get_velocity() << std::setw(15)
                    << mrc.get_current() << std::setw(18)
                    << mrc.get_collision_flag() << std::setw(10)
                    << mrc.get_mode_string() << std::endl;
        }
        std::cout << std::string(88, '-') << std::endl;
      }

      std::this_thread::sleep_until(next);
    }

    std::cout << "\nStopping... disabling all devices." << std::endl;
    safeguarder.disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } catch (const std::exception& e) {
    std::cerr << "[Safeguarder Exception]: " << port << ">> " << e.what()
              << std::endl;
    return -1;
  }

  return 0;
}


