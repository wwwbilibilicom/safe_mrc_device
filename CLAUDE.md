# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**SafeMRC Device Library** is a C++17 library for controlling Safe Magnetorheological Clutch (MRC) actuators over RS485 serial communication. It's designed for high-frequency (1000+ Hz) communication with multiple MRC devices on a single RS485 bus. This is part of the SafeArm robotic system at USTC.

### Key Characteristics
- **Real-time Performance**: Supports 1000 Hz control loops
- **Hardware Interface**: RS485 serial at 4 Mbps
- **Thread-Safe**: Concurrent read/write with atomic synchronization
- **No FIFO on Slaves**: Current branch (no_fifo_version) compensates for slave devices lacking FIFO buffers with configurable delays

## Build Commands

### Standard Build
```bash
cd /home/wenbo/lab_dev/safe_mrc_device
mkdir -p build && cd build
cmake ..
make -j4
```

### Install Library
```bash
sudo make install
```

### Run Examples
```bash
# Basic demo (requires /dev/ttyUSB0)
sudo ./demo

# 1000Hz pressure test with bus detection (requires /dev/ttyUSB1)
sudo ./pressure_test_1000Hz
```

### Dependencies
- **libserial**: System package or local copy in `lib/`
- **CMake**: 3.22+
- **Compiler**: C++17 support (GCC/Clang)
- **Hardware**: RS485 to USB converter

## Architecture Overview

### 8-Layer Communication Stack

```
Layer 8: Safeguarder (High-level API)
   ↓
Layer 7: SafeMRCComponent (Device Collection Management)
   ↓
Layer 6: SafeMRCDeviceCollection (Bulk Operations)
   ↓
Layer 5: MRC (Device State Container)
   ↓
Layer 4: SafeMRCRS485Device (Protocol Adapter)
   ↓
Layer 3: RS485DeviceCollection (Frame Dispatch + Bus Detection)
   ↓
Layer 2: RS485Serial (RX Thread + Buffer Management)
   ↓
Layer 1: libserial (Hardware Communication)
```

### Critical Components

#### RS485 Communication Layer (`src/rs485bus/`)
- **RS485Serial**: Inherits from `serial::Serial`, manages 4 Mbps communication
  - Thread-safe write with `rx_busy_` atomic flag
  - Configurable rx_delay (default 200μs) for non-FIFO slave devices
  - Buffer management via `rx_buffer_` vector

- **RS485DeviceCollection**: Frame reception and dispatch
  - Background RX thread continuously reads from serial port
  - `unpackStream()`: Synchronizes on 0xFE 0xEE headers, validates CRC
  - Dispatches valid frames to registered devices by ID
  - **New**: Integrated `BusDetector` for real-time statistics

- **BusDetector** (NEW - recently added): Monitors communication health
  - Tracks success rate, response times per device
  - Non-intrusive: hooks into existing send/receive flow
  - Thread-safe with mutex-protected statistics

#### Device Abstraction Layer (`src/safe_mrc/`)
- **MRC**: Device state container (position, velocity, current, collision flag)
  - Thread-safe state updates via mutex
  - Unit conversions: current (mA→A), position (normalized 0-1), velocity (rad/s)

- **SafeMRCRS485Device**: Protocol adapter
  - Creates command frames with CRC
  - Parses feedback frames with validation
  - Inherits from `RS485Device` base class

- **SafeMRCDeviceCollection**: Bulk device operations
  - enable_all(), disable_all(), refresh_all(), set_zero_all()
  - Command broadcasting to multiple devices
  - **Integration point for BusDetector statistics**

- **Safeguarder**: Top-level API
  - Device initialization: `init_mrcs(types, ids)`
  - Network configuration: timeouts, delays
  - Unified control interface

### Communication Protocol

#### Command Frame (10 bytes)
```
Header  ID   Mode  Desired Current (4B)  CRC
0xFE 0xEE  u8   u8        int32         uint16
```

#### Feedback Frame (17 bytes)
```
Header  ID  Mode Coll  Position(4B) Velocity(4B) Current(2B)  CRC
0xFE 0xEE u8   u8   u8     int32        int32       int16    uint16
```

#### Operation Modes (MRCMode enum)
- **FREE (0)**: No torque control
- **FIX_LIMIT (1)**: Fixed torque limit
- **ADAPTATION (2)**: Adaptive control
- **DEBUG (3)**: Debug mode
- **RESET (4)**: Reset device
- **ZERO (5)**: Zero position reference
- **REFRESH (6)**: State query only (no control)

#### CRC Validation
- Algorithm: CRC-CCITT (0xFFFF initial value)
- Implementation: `src/rs485bus/crc_ccitt.h`
- Applied to all frames except final 2 CRC bytes

## Key Implementation Details

### Thread Safety Pattern
```cpp
// RS485Serial uses atomic for RX synchronization
std::atomic<bool> rx_busy_{false};

// Device state protected by mutex
std::mutex state_mutex_;
std::lock_guard<std::mutex> lock(state_mutex_);
```

### No-FIFO Compensation
The current branch (`no_fifo_version`) works with slave devices that lack FIFO buffers. Key mitigation:
- **rx_delay_us_**: Configurable delay after write (default 200μs)
- Prevents overwhelming slave device buffers
- Adjustable via `set_rx_delay_us()`

### Frame Synchronization
```cpp
// RS485DeviceCollection::unpackStream()
// 1. Search for 0xFE 0xEE header
// 2. Extract sizeof(MRCFdkFrame) bytes
// 3. Validate CRC
// 4. Dispatch to device by ID
// 5. Erase processed bytes from buffer
```

### Bus Detection Integration (NEW)
```cpp
// Send command (Layer 6)
detector->recordRequestStart(device_id);
serial.write_rs485_data(frame);

// Receive response (Layer 3)
detector->recordSuccess(device_id);  // Auto-calculates response time
dispatch_frame_callback(frame);
```

## Common Development Patterns

### Adding a New Device Type
1. Define device type in `MRCType` enum (safe_mrc.hpp)
2. Add device-specific parameters if needed
3. Update device initialization in Safeguarder
4. Handle type-specific logic in MRC class

### Extending Communication Protocol
1. Modify frame structures in `safe_mrc_protocol.h`
2. Update `create_cmd_frame()` in SafeMRCRS485Device
3. Update `parse_fdk_frame()` for new fields
4. Adjust `sizeof()` checks in unpackStream()
5. **Important**: Update CRC calculation boundaries

### Using Bus Detection
```cpp
// Enable detection
safeguarder.get_mrc_component().get_device_collection()
    .enable_bus_detection(true);

// Query statistics
auto* detector = collection.get_bus_detector();
DeviceStats stats = detector->getDeviceStats(device_id);
std::string report = detector->generateReport();

// Reset statistics
detector->resetStats();  // All devices
detector->resetStats(device_id);  // Specific device
```

### Debugging Communication Issues
1. Enable frame printing: `rs485_serial_.printf_frame((uint8_t*)&cmd_frame);`
2. Check `last_error_` strings in RS485Serial
3. Monitor CRC mismatch warnings in console output
4. Use BusDetector to identify problematic devices
5. Adjust timeouts: `set_rx_timeout_us()` and `set_rx_delay_us()`

## File Organization

### Headers (`include/safe_mrc_device/`)
```
rs485bus/
  ├── rs485_serial.hpp          - Serial communication
  ├── rs485_device_collection.hpp - Frame dispatch
  ├── bus_detector.hpp           - Statistics (NEW)
  └── crc_ccitt.h                - CRC validation

safe_mrc/
  ├── safeguarder.hpp            - Top-level API
  ├── safe_mrc_component.hpp     - Component management
  ├── safe_mrc_device_collection.hpp - Bulk operations
  ├── safe_mrc_device.hpp        - Protocol adapter
  ├── safe_mrc.hpp               - Device state
  └── safe_mrc_protocol.h        - Frame definitions
```

### Source (`src/`)
- Mirrors header structure
- Implementation details match header organization

### Examples & Tests
- `examples/demo.cpp`: Basic usage example
- `test/pressure_test_1000Hz.cpp`: High-frequency stress test with bus detection

## Important Notes

### Performance Considerations
- **1000 Hz operation**: Minimize work in control loop
- **Mutex contention**: BusDetector uses lock_guard, but operations are fast
- **Memory allocation**: Pre-allocate buffers, avoid heap in hot path
- **RX thread priority**: Consider real-time scheduling for production

### Known Issues & Mitigations
- **Double-free bug**: Fixed in commit 17a4213 / 804aa14
- **No FIFO on slaves**: Mitigated with rx_delay_us_
- **Device ID conflicts**: Ensure unique IDs across network
- **CRC mismatches**: Usually indicate noise or timing issues

### Hardware Requirements
- RS485 to USB converter (typically /dev/ttyUSB0 or /dev/ttyUSB1)
- MRC slave devices with proper RS485 termination
- Linux system with proper serial port permissions (sudo or udev rules)

## Recent Development (Current Branch: no_fifo_version)

### Recent Changes
- Removed FIFO dependency from slave communication
- Added bus detection and statistics module
- Fixed double-free corruption issues
- Tested at 1000 Hz with 2 devices
- **CRITICAL FIX (Phase 1)**: Fixed multi-slave frame latency issue in rxThread()

### Phase 1 Latency Fix (Latest)
**Problem**: In multi-slave scenarios, `rxThread()` called `unpackStream()` only once per loop iteration, causing subsequent frames to accumulate in buffer until new bytes arrived.

**Solution**: Modified `RS485DeviceCollection::rxThread()` (src/rs485bus/rs485_device_collection.cpp:88) to continuously drain buffer:
```cpp
// NEW: Drain all complete frames
while (unpackStream(rs485_serial_.rx_buffer_)) {
    // Continue unpacking until no complete frames remain
}
```

**Impact**:
- Eliminates frame latency in 2+ slave configurations
- Improves BusDetector response time accuracy
- Maintains 1000 Hz operation capability
- Backward compatible (no API changes)

### Integration Example
See `test/pressure_test_1000Hz.cpp` for complete integration:
- Bus detection enabled at startup
- Statistics printed every 1000 iterations (1 second at 1kHz)
- Success rates and response times monitored in real-time

# Hardware dependency

this lib depends on the a USB-RS485 converter, and the RS485 bus should be terminated with 120Ω resistor. The baudrate is 400000bps. The device is registered as /dev/ttyUSB0 or /dev/ttyUSB1.

