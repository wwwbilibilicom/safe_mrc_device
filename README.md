# SafeMRC Device RS485 Library

A C++ library for RS485 communication with SafeMRC (Safe Magneticrheological Clutch) device hardware. This library is a part of SafeArm. See detailed setup guide and docs in following updates.

## 🎯Quick Start

### 👀️Prerequisites

- RS485 to serial converter
- CMake 3.22+
- C++17 compiler

### 🚀️Build & Install

```bash
git clone https://github.com/wwwbilibilicom/safe_mrc_device.git
cd safe_mrc_device
mkdir build && cd build
cmake ..
make
make install
```

### 🔨Usage

```cpp
#include <safe_mrc_device/safe_mrc/safe_mrc.hpp>
#include <safe_mrc_device/safe_mrc/safeguarder.hpp>

std::string port = "/dev/ttyUSB0";
safe_mrc::Safeguarder safeguarder(port);
std::vector<safe_mrc::MRCType> mrc_types = {safe_mrc::MRCType::ROTARY96};
std::vector<uint8_t> rs485_ids = {0x01};
safeguarder.init_mrcs(mrc_types, rs485_ids);
safeguarder.enable_all{}

```

### 🔍Examples

- **C++**: examples/demo.cpp -Complete safeguarder control demo and start with command `sudo ./demo`.

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
- **REFRESH**: Only update current state without any control

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

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 🔑License

This package is distributed under the MIT License. See the LICENSE file for details.
