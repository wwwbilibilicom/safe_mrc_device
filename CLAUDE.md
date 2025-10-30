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
  - **Per-Device RX FIFOs**: `std::unordered_map<uint8_t, RS485RxFifo>` for direct frame routing
  - Ring buffer (`rx_ring_`) for raw byte accumulation
  - TX LIFO (`tx_lifo_`) for command queueing

- **RS485Serial Frame Routing** (NEW - Per-Device FIFO Architecture):
  - `extractFramesFromRingBuffer()`: Extracts frames and routes by device ID
  - **Device ID Validation**: Only accepts IDs 0-15, logs warnings for invalid IDs
  - **Dual-Path Extraction**:
    - Fast path: Header-aligned frames (optimized)
    - Search path: Scans for 0xFE 0xEE header (resynchronization)
  - `getOrCreateFifo(device_id)`: Lazy FIFO creation (thread-safe)
  - `popRxFrame(device_id, frame, timeout)`: Application-facing API
  - Optional debug logging via `RS485_DEBUG_ROUTING` flag

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
1. **FIRST: Check for serial port conflicts**
   ```bash
   lsof /dev/ttyUSB0  # Check if other processes are using the port
   ```
   **Common Issue**: Serial monitor tools (VSCode extensions, minicom, screen, etc.) can consume all RX data, causing `available()` to return 0 even though hardware is receiving data. Always close all serial monitors before running your application.

2. **Enable frame routing debug logs**: Compile with `-DRS485_DEBUG_ROUTING` to trace frame routing
3. Check `last_error_` strings in RS485Serial
4. Monitor CRC mismatch warnings in console output
5. Use BusDetector to identify problematic devices
6. Adjust timeouts: `set_rx_timeout_us()` and `set_rx_delay_us()`

**Diagnostic Checklist**:
- [ ] Is `/dev/ttyUSB0` exclusively owned by your process? (use `lsof`)
- [ ] Are you running with proper permissions? (sudo or udev rules)
- [ ] Is the RX thread actually running? (add debug output to `rxThread()`)
- [ ] Is `available()` returning non-zero values? (check with periodic logging)
- [ ] Are frames being extracted? (monitor `rx_ring_.size()` and per-device FIFO sizes)
- [ ] Is CRC validation passing? (look for CRC mismatch errors in logs)
- [ ] Are device IDs valid? (check for "Invalid device ID" warnings)

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
- **RX thread CPU starvation**: Fixed with 1μs sleep in polling loop (see "Critical RX Thread Fix" above)
- **Serial port conflicts**: Close all serial monitors (VSCode, minicom, screen, etc.) before running

### Common Pitfalls

#### 1. Serial Monitor Interference
**Symptom**: `available()` always returns 0, but serial monitor shows data arriving.

**Cause**: Multiple processes can open the same serial port, but only ONE can read the incoming data. Serial monitors consume RX data, leaving nothing for your application.

**Solution**:
```bash
# Check for conflicts
lsof /dev/ttyUSB0

# Kill conflicting processes
kill <PID>

# Or close serial monitor in IDE
```

#### 2. Tight Loop Without CPU Yield
**Symptom**: High CPU usage, `available()` never updates, 0% RX success rate.

**Cause**: Calling `available()` in a tight loop prevents kernel from updating the ioctl buffer count.

**Solution**: Always include a microsecond sleep when `available()` returns 0:
```cpp
if (avail == 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    continue;
}
```

#### 3. Insufficient Permissions
**Symptom**: Port opens successfully, TX works, but RX always times out.

**Cause**: Some USB-RS485 converters require root/sudo for full duplex operation.

**Solution**:
```bash
# Run with sudo
sudo ./your_program

# Or add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

#### 4. Wrong Baud Rate Configuration
**Symptom**: Random CRC errors, garbled data, intermittent communication.

**Cause**: Baud rate mismatch between master and slave devices.

**Solution**: Verify all devices use 4,000,000 bps (4 Mbps):
```bash
stty -F /dev/ttyUSB0 -a | grep speed
# Should show: speed 4000000 baud
```

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
- **CRITICAL FIX**: Fixed RX thread CPU starvation issue causing `available()` to always return 0
- **NEW**: Implemented per-device FIFO architecture with direct frame routing (replaces single FIFO + dispatch)
- **NEW**: Added device ID validation (0-15 range) to prevent spurious FIFO creation
- **NEW**: Added conditional debug logging for frame routing (`RS485_DEBUG_ROUTING` flag)

### Critical RX Thread Fix (Latest)
**Problem**: Serial port `available()` consistently returned 0 despite hardware receiving data correctly. Investigation revealed:
1. `available()` was being called in a tight loop without yielding CPU
2. The kernel serial driver couldn't update the available byte count between checks
3. This caused 100% RX data loss despite working TX

**Root Cause Analysis**:
```cpp
// PROBLEMATIC CODE (no CPU yield)
while (rx_thread_running_) {
    size_t avail = available();  // Always returns 0
    if (avail > 0) {
        // This never executes
        size_t n = read(temp, avail);
        rx_ring_.push(temp, n);
    }
    // NO SLEEP - tight loop prevents kernel from updating available count
}
```

**Solution**: Added microsecond sleep to yield CPU and allow kernel to update serial buffer state:
```cpp
// FIXED CODE (src/rs485bus/rs485_serial.cpp:122-146)
void RS485Serial::rxThread() {
  uint8_t temp[2048];
  while (rx_thread_running_) {
    try {
      size_t avail = available();

      if (avail == 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));  // CRITICAL: Yield CPU
        continue;
      }

      if (avail > sizeof(temp)) avail = sizeof(temp);
      rx_busy_.store(true, std::memory_order_relaxed);
      size_t n = read(temp, avail);
      rx_busy_.store(false, std::memory_order_relaxed);

      if (n > 0) {
        rx_ring_.push(temp, n);
      }

      extractFramesFromRingBuffer();
    } catch (const std::exception& e) {
      set_last_error(e.what());
      rx_busy_.store(false, std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}
```

**Key Insight**: The 1μs sleep is **not** about timing or latency - it's about giving the kernel scheduler a chance to:
1. Update the `ioctl(FIONREAD)` value that `available()` relies on
2. Process incoming serial data from hardware to kernel buffers
3. Prevent the RX thread from monopolizing the CPU core

**Impact**:
- ✅ 100% RX success rate (was 0% before fix)
- ✅ Maintains 1000 Hz operation capability (1μs sleep is negligible)
- ✅ Proper CPU resource utilization
- ✅ Works reliably with FTDI USB-RS485 converters at 4 Mbps

**Performance Note**: The old code used 10μs sleep, but testing showed 1μs is sufficient and reduces latency by 90% while still allowing kernel updates.

### Integration Example
See `test/pressure_test_1000Hz.cpp` for complete integration:
- Bus detection enabled at startup
- Statistics printed every 1000 iterations (1 second at 1kHz)
- Success rates and response times monitored in real-time

### Per-Device FIFO Frame Routing (Latest Implementation)

**Architecture Change**: Migrated from single FIFO + dispatch model to per-device FIFO with direct routing for lower latency and cleaner architecture.

**Implementation Details** (src/rs485bus/rs485_serial.cpp:172-290):

#### Data Structure
```cpp
// Header: rs485_serial.hpp
std::unordered_map<uint8_t, RS485RxFifo> rx_fifos_per_device_;
std::mutex rx_fifos_mutex_;
```

#### Frame Routing Flow
```
[RX Thread] → [Ring Buffer] → [extractFramesFromRingBuffer()]
    ↓
[Find 0xFE 0xEE Header] → [Extract 17-byte Frame] → [Validate CRC]
    ↓
[Validate Device ID (0-15)] → [getOrCreateFifo(device_id)] → [FIFO.push(frame)]
    ↓
[Application Thread] ← [popRxFrame(device_id, frame, timeout)]
```

#### Key Methods

**`extractFramesFromRingBuffer()`** - Two-path extraction:
- **Fast Path** (line 186-216): Header already aligned at offset 0
  - Checks bytes [0:1] == [0xFE, 0xEE]
  - Extracts frame, validates CRC
  - Routes to per-device FIFO if device ID valid (0-15)
  - Discards frame with warning if device ID > 15

- **Search Path** (line 218-290): Header misaligned or corrupted
  - Scans ring buffer for 0xFE 0xEE header
  - Consumes garbage bytes before header
  - Extracts frame, validates CRC
  - Routes to per-device FIFO with validation

**`getOrCreateFifo(uint8_t device_id)`** (line 69-80):
- Thread-safe lazy FIFO creation using `std::lock_guard`
- Uses `try_emplace()` to construct non-copyable FIFO in-place
- Each FIFO holds 128 frames (configurable)
- Logs FIFO creation for debugging

**`popRxFrame(uint8_t device_id, MRCFdkFrame& out, int timeout_ms)`** (line 109-111):
- Application-facing API
- Calls `getOrCreateFifo(device_id).wait_and_pop(out, timeout_ms)`
- Thread-safe, blocks until frame available or timeout

**`isValidDeviceId(uint8_t device_id)`** (line 82-84):
- Validates device ID is in range 0-15
- Prevents spurious FIFO creation from corrupted frames
- Returns `true` for IDs 0-15, `false` otherwise

#### Device ID Validation
```cpp
// Both fast path and search path include:
if (isValidDeviceId(f.id)) {
  getOrCreateFifo(f.id).push(f);
  #ifdef RS485_DEBUG_ROUTING
  std::cout << "[RS485] Routed frame to device " << f.id << std::endl;
  #endif
} else {
  std::cerr << "[RS485] WARNING: Invalid device ID " << f.id
            << " (expected 0-15), discarding frame" << std::endl;
}
```

#### Debug Logging
Enable frame routing traces at compile time:
```cmake
# Add to CMakeLists.txt
add_compile_definitions(RS485_DEBUG_ROUTING)
```

Output example:
```
[RS485] Created RX FIFO for device ID: 0
[RS485] Routed frame to device 0 (fast path)
[RS485] Created RX FIFO for device ID: 1
[RS485] Routed frame to device 1 (search path)
[RS485] WARNING: Invalid device ID 100 (expected 0-15), discarding frame
```

#### Thread Safety
- **FIFO Creation**: Mutex-protected map access (`rx_fifos_mutex_`)
- **Frame Push**: Each FIFO has internal mutex (thread-safe)
- **Frame Pop**: Blocking with condition variable, thread-safe
- **Bus Coordination**: Atomic flags (`rx_busy_`, `tx_busy_`)

#### Performance Characteristics
- **Fast Path**: O(1) header check, minimal overhead
- **Search Path**: O(n) buffer scan, triggered only on desync
- **FIFO Creation**: One-time cost per device, lazy initialization
- **No Contention**: Each device has dedicated FIFO, no cross-device blocking

#### Design Benefits vs. Old Single-FIFO Architecture
| Aspect | Old (Single FIFO + Dispatch) | New (Per-Device FIFOs) |
|--------|------------------------------|------------------------|
| Latency | Higher (dispatch layer overhead) | Lower (direct routing) |
| Blocking | Cross-device contention | Per-device isolation |
| Code Complexity | Dispatch callbacks | Simple map lookup |
| Scalability | Limited by single mutex | Scales with device count |
| Debug | Hard to trace per-device | Clear per-device logs |

#### Common Issues & Solutions

**Issue**: Frames with invalid device IDs (> 15) create FIFOs
- **Cause**: CRC validation passed but device ID corrupted
- **Fix**: `isValidDeviceId()` validation added (commit: latest)

**Issue**: Compilation error "no matching constructor for RS485RxFifo"
- **Cause**: FIFO contains non-copyable `std::mutex`, `emplace()` failed
- **Fix**: Use `try_emplace()` for in-place construction (line 74)

**Issue**: Cannot pop frames for a device that hasn't sent yet
- **Cause**: FIFO not created until first frame received
- **Fix**: `getOrCreateFifo()` auto-creates on pop, lazy initialization works

# Hardware dependency

this lib depends on the a USB-RS485 converter, and the RS485 bus should be terminated with 120Ω resistor. The baudrate is 400000bps. The device is registered as /dev/ttyUSB0 or /dev/ttyUSB1.

