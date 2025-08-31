#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <safe_mrc_device/safe_mrc/safeguarder.hpp>
#include <safe_mrc_device/safe_mrc/safe_mrc.hpp>
#include <thread>


int	main(int argc, char **argv)
{
    try{
        std::cout << "=== SafeMRC Demo ===" << std::endl;
        std::cout << "This demo shows the SafeMRC API functionality" << std::endl;

        // Initialize Safeguarder with RS485 port
        std::cout << "Initializing Safeguarder..." << std::endl;
        safe_mrc::Safeguarder safeguarder("/dev/ttyUSB0");

        // Initialize MRC devices
        std::vector<safe_mrc::MRCType> mrc_types = {safe_mrc::MRCType::ROTARY96, safe_mrc::MRCType::ROTARY52};
        std::vector<uint8_t> rs485_ids = {0x01, 0x02};
        safeguarder.init_mrcs(mrc_types, rs485_ids);

        // Enable all MRCs
        std::cout << "\n=== Enabling MRCs ===" << std::endl;
        safeguarder.enable_all();
        // Allow time (2ms) for the MRCs to respond for slow operations like enabling
        safeguarder.set_rx_timeout_us(2000);

        // Access MRCs through component
        for (const auto& mrc : safeguarder.get_mrc_component().get_mrcs()) {
            std::cout << "MRC RS485 ID: " << static_cast<int>(mrc.get_rs485_id())
                      << " Type: " << (mrc.get_mrc_type() == safe_mrc::MRCType::ROTARY96 ? "ROTARY96" : "ROTARY52")
                      << std::endl;
        }

        // Control MRCs
        std::cout << "\n=== Controlling MRCs ===" << std::endl;
        safeguarder.get_mrc_component().mrc_control_all({safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.5},
                                                        safe_mrc::MRCCmd{safe_mrc::MRCMode::FIX_LIMIT, 0.8}});

        for (int i = 0; i < 10; i++)
        {       
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            safeguarder.refresh_all();
            for(const auto& mrc : safeguarder.get_mrc_component().get_mrcs()){
                std::cout << "MRC RS485 ID: " << mrc.get_rs485_id()
                          << " Position: " << mrc.get_position()
                          << " Velocity: " << mrc.get_velocity()
                          << " Current: " << mrc.get_current()
                          << " Collision Flag: " << mrc.get_collision_flag()
                          << " Mode: " << mrc.get_mode_string()
                          << std::endl;
            }

        }

        safeguarder.disable_all();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
