#include "safe_mrc_device/rs485bus/rs485_serial.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace safe_mrc;

int main()
{

    RS485Serial rs485("/dev/ttyUSB0");
    try
    {
        while (true)
        {
            // Example: Send a command frame
            MRCCmdFrame cmd_frame;
            cmd_frame.head[0] = 0xFE;
            cmd_frame.head[1] = 0xEE;
            cmd_frame.id = 0x01; // Device ID
            cmd_frame.mode = static_cast<uint8_t>(MRCMode::FIX_LIMIT);
            int32_t current = static_cast<int32_t>(std::round(0.5 * 1000.0f));
            cmd_frame.des_coil_current = current;
            cmd_frame.CRC16Data =
                crc_ccitt(0xFFFF, reinterpret_cast<uint8_t *>(&cmd_frame), sizeof(cmd_frame) - 2);

            rs485.pushTxFrame(cmd_frame);
            rs485.printTxFrameHex(cmd_frame);
            

            // Example: Receive a feedback frame
            if (rs485.printRxFrame())
            {
            }
            else
            {
                std::cout << "No feedback received within timeout." << std::endl;
            }

            std::cout << "----------------------------------------" << std::endl
                      << std::endl;
            // rs485.showRxRingBuffer();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RS485Serial Exception]: " << e.what() << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return 0;
}
