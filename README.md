# SafeMRC Device Package

## Overview

The `safe_mrc_device` package provides a comprehensive framework for managing SafeMRC (Safe Magnetic Rheological Clutch) devices through RS485 communication. This package is designed for robotic applications requiring precise torque control and collision detection.

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Safeguarder   │───▶│ MRCComponent     │───▶│ SafeMRCRS485    │
│   (Main API)    │    │ (Device Manager) │    │ (Device Driver) │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│RS485DeviceColl  │    │   MRC Device     │    │   RS485 Bus     │
│(Communication)  │    │  (State Mgmt)    │    │ (Serial Comm)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Key Components

### 1. MRC Class
- **Purpose**: Manages individual MRC device state
- **Features**: 
  - Position, velocity, current monitoring
  - Collision flag detection
  - Multiple operation modes
  - Device type support (ROTARY96, ROTARY52, LINEAR)

### 2. SafeMRCRS485Device Class
- **Purpose**: RS485 communication driver for MRC devices
- **Features**:
  - Command frame creation
  - Feedback frame parsing
  - CRC validation
  - Error handling

### 3. SafeMRCDeviceCollection Class
- **Purpose**: Manages multiple MRC devices
- **Features**:
  - Bulk operations (enable_all, disable_all, etc.)
  - Individual device control
  - Device monitoring and status

### 4. Safeguarder Class
- **Purpose**: High-level interface for MRC networks
- **Features**:
  - Device initialization
  - Network configuration
  - Unified control interface

### 5. RS485 Communication Layer
- **Purpose**: Low-level serial communication
- **Features**:
  - 4Mbps communication
  - Timeout handling
  - Thread-safe operations
  - CRC validation

## Installation

### Prerequisites
- ROS 2 (Humble or later)
- C++17 compatible compiler
- CMake 3.22 or later

### Dependencies
```bash
# Install serial library
sudo apt install ros-humble-serial

# Or build from source
git clone https://github.com/wjwwood/serial.git
cd serial && mkdir build && cd build
cmake .. && make && sudo make install
```

### Building
```bash
# Navigate to workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select safe_mrc_device

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Example

```cpp
#include "safe_mrc_device/safe_mrc/safeguarder.hpp"
#include <iostream>

int main() {
    try {
        // Initialize Safeguarder with serial port
        safe_mrc::Safeguarder safeguarder("/dev/ttyUSB0");
        
        // Configure MRC devices
        std::vector<safe_mrc::MRCType> mrc_types = {
            safe_mrc::MRCType::ROTARY96, 
            safe_mrc::MRCType::ROTARY52
        };
        std::vector<uint8_t> rs485_ids = {1, 2};
        
        safeguarder.init_mrcs(mrc_types, rs485_ids);
        
        // Enable all devices
        safeguarder.enable_all();
        
        // Control individual device
        auto& mrc_component = safeguarder.get_mrc_component();
        mrc_component.mrc_control_one(0, {
            static_cast<uint8_t>(safe_mrc::MRCMode::ADAPTATION), 
            0.5f  // 0.5A current
        });
        
        // Monitor device status
        auto mrcs = mrc_component.get_mrcs();
        for (const auto& mrc : mrcs) {
            std::cout << "Device " << static_cast<int>(mrc.get_rs485_id()) 
                      << " Position: " << mrc.get_position() 
                      << " Collision: " << static_cast<int>(mrc.get_collision_flag()) 
                      << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### Advanced Usage

```cpp
// Multiple device collections
safe_mrc::Safeguarder safeguarder("/dev/ttyUSB0");

// Initialize different device groups
std::vector<safe_mrc::MRCType> arm_mrcs = {safe_mrc::MRCType::ROTARY96, safe_mrc::MRCType::ROTARY96};
std::vector<uint8_t> arm_ids = {1, 2};
safeguarder.init_mrcs(arm_mrcs, arm_ids);

// Set communication timeout
safeguarder.set_rx_timeout_us(100);  // 100 microseconds

// Bulk operations
safeguarder.enable_all();
safeguarder.refresh_all();

// Individual control with different modes
auto& mrc_component = safeguarder.get_mrc_component();
mrc_component.mrc_control_one(0, {static_cast<uint8_t>(safe_mrc::MRCMode::FIX_LIMIT), 1.0f});
mrc_component.mrc_control_one(1, {static_cast<uint8_t>(safe_mrc::MRCMode::ADAPTATION), 0.8f});
```

## Communication Protocol

### Command Frame (8 bytes)
```
┌─────┬─────┬─────┬─────┬─────────────┬─────┐
│ 0xFE│ 0xEE│ ID  │Mode │   Current   │ CRC │
│     │     │     │     │   (4 bytes) │     │
└─────┴─────┴─────┴─────┴─────────────┴─────┘
```

### Feedback Frame (17 bytes)
```
┌─────┬─────┬─────┬─────┬─────┬─────────────┬─────────────┬─────────────┬─────┐
│ 0xFE│ 0xEE│ ID  │Mode │Coll │  Position   │  Velocity   │   Current   │ CRC │
│     │     │     │     │Flag │ (4 bytes)   │ (2 bytes)   │ (2 bytes)   │     │
└─────┴─────┴─────┴─────┴─────┴─────────────┴─────────────┴─────────────┴─────┘
```

### Operation Modes
- **FREE**: No torque control
- **FIX_LIMIT**: Fixed torque limit
- **ADAPTATION**: Adaptive torque control
- **DEBUG**: Debug mode
- **RESET**: Reset device
- **ZERO**: Zero position reference

## Safety Features

### Collision Detection
- Real-time monitoring of collision flags
- Automatic collision state reporting
- Configurable collision thresholds

### Communication Safety
- CRC-CCITT validation for all frames
- Timeout protection
- Error recovery mechanisms
- Thread-safe operations

### Error Handling
- Comprehensive error reporting
- Exception handling
- Graceful degradation
- Logging and debugging support

## Configuration

### Serial Port Settings
- **Baud Rate**: 4,000,000 bps
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Timeout**: Configurable (default: 20μs)

### Device Configuration
- **RS485 IDs**: Unique identifiers for each device
- **MRC Types**: Device-specific configuration
- **Operation Modes**: Runtime mode selection
- **Current Limits**: Configurable torque limits

## Troubleshooting

### Common Issues

1. **Serial Port Access Denied**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Reboot or logout/login
   ```

2. **Communication Timeouts**
   - Check cable connections
   - Verify device IDs
   - Adjust timeout settings
   - Check for electrical interference

3. **CRC Errors**
   - Verify cable quality
   - Check termination resistors
   - Ensure proper grounding
   - Verify device configuration

4. **Build Errors**
   ```bash
   # Clean build
   colcon build --clean
   # Rebuild
   colcon build --packages-select safe_mrc_device
   ```

### Debug Mode
```cpp
// Enable debug output
mrc_component.mrc_control_one(0, {static_cast<uint8_t>(safe_mrc::MRCMode::DEBUG), 0.0f});

// Check device status
auto mrc = mrc_component.get_mrcs()[0];
std::cout << "Mode: " << mrc.get_mode_string() << std::endl;
std::cout << "Enabled: " << mrc.is_enabled() << std::endl;
```

## API Reference

### Safeguarder Class
- `Safeguarder(const std::string& port)`
- `void init_mrcs(const std::vector<MRCType>& mrc_types, const std::vector<uint8_t>& rs485_ids)`
- `void enable_all()`
- `void disable_all()`
- `void refresh_all()`
- `void set_zero_all()`
- `void set_rx_timeout_us(int timeout_us)`

### MRCComponent Class
- `void init_mrc_devices(std::vector<MRCType> mrc_types, std::vector<uint8_t> rs485_ids)`
- `void mrc_control_one(int i, const MRCCmd& cmd)`
- `void mrc_control_all(const std::vector<MRCCmd>& cmds)`
- `std::vector<MRC> get_mrcs() const`

### MRC Class
- `bool is_enabled() const`
- `double get_position() const`
- `double get_velocity() const`
- `double get_current() const`
- `uint8_t get_collision_flag() const`
- `std::string get_mode_string() const`
- `void set_enabled(bool enable)`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This package is distributed under the MIT License. See the LICENSE file for details.

## Support

- **Documentation**: [OpenArm Docs](https://docs.openarm.dev)
- **Community**: [Discord](https://discord.gg/FsZaZ4z3We)
- **Contact**: <openarm@enactic.ai>
- **Issues**: GitHub Issues page 