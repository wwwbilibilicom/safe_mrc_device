#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <safe_mrc_device/safe_mrc/safe_mrc.hpp>
#include <safe_mrc_device/safe_mrc/safeguarder.hpp>
#include <thread>

int main() {
  std::string port = "/dev/ttyUSB0";
  try {
    std::cout << "=== SafeMRC Demo ===" << std::endl;
    std::cout << "This demo shows the SafeMRC API functionality" << std::endl;

    // Initialize Safeguarder with RS485 port
    std::cout << "Initializing Safeguarder..." << std::endl;
    safe_mrc::Safeguarder safeguarder(port);

    // Initialize MRC devices
    std::vector<safe_mrc::MRCType> mrc_types = {
        safe_mrc::MRCType::ROTARY52, safe_mrc::MRCType::ROTARY52,
        safe_mrc::MRCType::ROTARY52, safe_mrc::MRCType::ROTARY52};
    std::vector<uint8_t> rs485_ids = {0x01, 0x02, 0x03, 0x04};
    safeguarder.init_mrcs(mrc_types, rs485_ids);

    // Enable all MRCs
    std::cout << "\n=== Enabling MRCs ===" << std::endl;
    safeguarder.enable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Allow time (2ms) for the MRCs to respond for slow operations like
    // enabling
    safeguarder.set_rx_timeout_us(2000);

    // Access MRCs through component
    for (const auto& mrc : safeguarder.get_mrc_component().get_mrcs()) {
      std::cout << "MRC RS485 ID: " << static_cast<int>(mrc.get_rs485_id())
                << " Type: "
                << (mrc.get_mrc_type() == safe_mrc::MRCType::ROTARY96
                        ? "ROTARY96"
                        : "ROTARY52")
                << std::endl;
    }

    // Control MRCs
    std::cout << "\n=== Controlling MRCs ===" << std::endl;
    safeguarder.get_mrc_component().mrc_control_all(
        {safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f},
         safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.0f}});

    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      safeguarder.refresh_all();
      // set print format
      std::cout << std::fixed
                << std::setprecision(2);  // set decimal precision to 2
      std::cout << std::left;             // left align text

      // print table header
      std::cout << std::setw(15) << "MRC RS485 ID" << std::setw(15)
                << "Position" << std::setw(15) << "Velocity" << std::setw(15)
                << "Current" << std::setw(18) << "Collision Flag"
                << std::setw(10) << "Mode" << std::endl;

      // print separator line
      std::cout << std::string(88, '-') << std::endl;

      for (const auto& mrc : safeguarder.get_mrc_component().get_mrcs()) {
        std::cout << std::setw(15) << static_cast<int>(mrc.get_rs485_id())
                  << std::setw(15) << mrc.get_position() << std::setw(15)
                  << mrc.get_velocity() << std::setw(15) << mrc.get_current()
                  << std::setw(18) << mrc.get_collision_flag() << std::setw(10)
                  << mrc.get_mode_string() << std::endl;
      }
      // print separator line
      std::cout << std::string(88, '-') << std::endl << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    safeguarder.disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } catch (const std::exception& e) {
    std::cerr << "[Safeguarder Exception]: " << port << ">> " << e.what()
              << std::endl;
    return -1;
  }
  return 0;
}
