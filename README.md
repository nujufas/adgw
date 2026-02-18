# Simulation Middleware - Phase 1: Data Structures

## Overview

This is **Phase 1** of the simulation middleware implementation, focusing on the canonical data structures that form the foundation of the entire architecture.

## What's Implemented

### Data Structures (middleware/data/)

All structures are designed for efficient binary serialization and network transmission:

1. **VehicleState** (`vehicle_state.h`) - **192 bytes**
   - Identity (id, timestamp)
   - Pose (x, y, z, heading, pitch, roll)
   - Velocity (vx, vy, vz, speed, acceleration, yaw_rate)
   - Dimensions (length, width, height, wheelbase)
   - Vehicle state (steering, throttle, brake, gear)
   - Status flags (type, control mode, lights)

2. **SceneState** (`scene_state.h`)
   - Weather conditions (temperature, visibility, precipitation, wind)
   - Road conditions (friction, curvature, grade, speed limit)
   - Lighting (time of day, ambient light)
   - Traffic context (density, vehicle/pedestrian counts)

3. **ControlCommand** (`control_command.h`) - **≤512 bytes**
   - Multiple control modes: SPEED, ACCELERATION, POSITION, TRAJECTORY, etc.
   - Mode-specific data (speed control, trajectory waypoints, etc.)
   - Validation and safety flags

4. **SensorData** (`sensor_data.h`)
   - Support for multiple sensor types (Camera, LiDAR, Radar, GPS, IMU)
   - Sensor-specific data formats
   - Detection data for radar/vision systems

### Serialization Utilities (middleware/utils/)

- **serialization.h**: Binary serialization/deserialization helpers
  - Single structure serialization
  - Vector serialization (for multiple vehicles)
  - Zero-copy memcpy-based implementation

### Unit Tests (tests/)

Comprehensive test suite covering:
- Structure size verification
- Default construction
- Set/get operations
- Equality operators
- Serialization/deserialization round-trips
- Vector serialization

## Build Instructions

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install build-essential unzip curl

# Bazel (if not already installed)
# Visit https://bazel.build/install for installation instructions
```

### Setup Third-Party Dependencies

Before building, download and extract the esmini simulator library:

```bash
# Run the setup script to download esmini
./scripts/setup_esmini.sh
```

This will download esmini v2.59.0 from GitHub and extract it to `third_party/esmini/`.

**Note**: The esmini files are not committed to the repository. You must run the setup script after cloning.

### Building

```bash
# Build all targets
bazel build //...

# Run tests
bazel test //tests:test_data_structures --test_output=all

# Run size report
bazel run //tests:size_report

# Clean build
bazel clean
```

### Expected Output

```
========================================
Middleware Data Structures - Unit Tests
========================================

Running VehicleState.SizeCheck... PASSED
Running VehicleState.DefaultConstruction... PASSED
Running VehicleState.SetAndGet... PASSED
Running VehicleState.Equality... PASSED
Running VehicleState.Serialization... PASSED
Running VehicleState.VectorSerialization... PASSED
Running SceneState.DefaultConstruction... PASSED
Running SceneState.SetAndGet... PASSED
Running SceneState.Serialization... PASSED
Running ControlCommand.SizeCheck... PASSED
Running ControlCommand.DefaultConstruction... PASSED
Running ControlCommand.SpeedControl... PASSED
Running ControlCommand.PositionControl... PASSED
Running ControlCommand.TrajectoryControl... PASSED
Running ControlCommand.Serialization... PASSED
Running SensorData.DefaultConstruction... PASSED
Running SensorData.GPSData... PASSED
Running SensorData.Serialization... PASSED

========================================
All tests PASSED!
========================================
```

## Project Structure

```
.
├── BUILD                       # Root BUILD file
├── MODULE.bazel                # Bazel module configuration (Bzlmod)
├── WORKSPACE                   # Bazel workspace (legacy)
├── .bazelrc                    # Bazel configuration
├── README.md                   # This file
├── MIDDLEWARE_DESIGN.md        # Complete architecture design
├── IMPLEMENTATION_PLAN.md      # Full implementation plan
│
├── middleware/
│   ├── BUILD                   # Middleware aggregation
│   ├── data/                   # Canonical data structures
│   │   ├── BUILD               # Data structures BUILD file
│   │   ├── vehicle_state.h     # VehicleState (192 bytes)
│   │   ├── scene_state.h       # SceneState
│   │   ├── control_command.h   # ControlCommand (≤512 bytes)
│   │   └── sensor_data.h       # SensorData
│   │
│   └── utils/                  # Utilities
│       ├── BUILD               # Utils BUILD file
│       └── serialization.h     # Binary serialization helpers
│
└── tests/
    ├── BUILD                   # Test targets
    ├── test_data_structures.cpp # Unit tests
    └── size_report.cpp         # Size verification
```

## Usage Example

```cpp
#include "middleware/data/vehicle_state.h"
#include "middleware/data/control_command.h"
#include "middleware/utils/serialization.h"

using namespace middleware::data;
using namespace middleware::utils;

// Create vehicle state
VehicleState vehicle;
vehicle.id = 1;
vehicle.x = 100.0;
vehicle.y = 50.0;
vehicle.speed = 20.0;

// Serialize to binary
uint8_t buffer[1024];
size_t size = serialize(vehicle, buffer, sizeof(buffer));

// Send over network (UDP, TCP, etc.)
// send(socket, buffer, size, 0);

// Deserialize
VehicleState received;
deserialize(buffer, size, received);

// Create control command
ControlCommand cmd;
cmd.vehicleId = 1;
cmd.mode = ControlCommand::Mode::SPEED;
cmd.speed.targetSpeed = 25.0;
cmd.speed.maxAcceleration = 2.0;
```

## Key Design Principles

1. **Fixed-Size Structures**: Optimized for network transmission
   - VehicleState: exactly 192 bytes
   - ControlCommand: ≤512 bytes

2. **Zero-Copy Serialization**: Direct memcpy for maximum performance

3. **Simulator-Independent**: No dependencies on esmini, CarMaker, or other simulators

4. **Language-Agnostic**: Binary format works with Python, MATLAB, C++, JavaScript, etc.

5. **Future-Proof**: Reserved fields for expansion without breaking compatibility

## Success Criteria ✓

- [x] All data structures compile and pass unit tests
- [x] VehicleState is exactly 192 bytes
- [x] ControlCommand is ≤512 bytes
- [x] Serialization/deserialization works correctly
- [x] No dependencies on external libraries (except C++ standard library)
- [x] Memory layout verified (alignment, padding)

## Next Steps (Phase 2)

The next phase will implement:
- Mock simulator adapter (simple kinematic vehicle model)
- Implements `ISimulator` interface
- Responds to control commands
- Generates synthetic vehicle data

See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for the complete roadmap.

## License

Copyright © 2026
