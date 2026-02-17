# Phase 5: Custom Test Application - ACC

## Overview

Phase 5 implements a Python-based ACC (Adaptive Cruise Control) test application to validate the complete middleware's closed-loop communication:

- **Middleware** (C++): Runs mock simulator, publishes perception data, receives control commands
- **ACC Application** (Python): Receives data, computes ACC logic, sends control commands back

## Components

### 1. Middleware Protocol (`apps/middleware_protocol.py`)

Binary serialization utilities matching C++ data structures:

- **VehicleState**: 192 bytes (id, position, velocity, dimensions, etc.)
- **ControlCommand**: ~468 bytes (header, control modes, parameters)

```python
from middleware_protocol import parse_vehicle_data, ControlCommand

# Parse received vehicles
vehicles = parse_vehicle_data(udp_data)

# Create speed command
cmd = ControlCommand.create_speed_command(vehicle_id=0, target_speed=20.0)
data = cmd.serialize()
```

### 2. ACC Application (`apps/test_acc.py`)

Simple ACC controller:

**Inputs:**
- Vehicle data via UDP multicast (239.255.0.1:48198)

**Logic:**
```
if no_lead_vehicle:
    target_speed = cruise_speed (20 m/s)
else:
    distance = lead_x - ego_x
    desired_distance = ego_speed * time_gap (2.0s)
    
    if too_close:
        decelerate
    elif too_far:
        accelerate
    else:
        match_lead_speed
```

**Outputs:**
- Control commands via UDP (localhost:53995)

## Running the Test

### Option 1: Automated Test Script

```bash
# Run complete closed-loop test
./test_phase5.sh
```

This script:
1. Builds the middleware
2. Starts `simple_demo` (middleware + mock simulator)
3. Starts `test_acc.py` (ACC application)
4. Monitors both processes
5. Cleans up on Ctrl+C

### Option 2: Manual Testing

**Terminal 1 - Start Middleware:**
```bash
bazel run //examples:simple_demo
```

**Terminal 2 - Start ACC Application:**
```bash
cd apps
python3 test_acc.py
```

**Terminal 3 - Monitor (optional):**
```bash
# Watch UDP traffic
sudo tcpdump -i lo -n udp port 48198 or udp port 53995
```

## Expected Behavior

The ACC application should:

1. **Receive Data**: ~100 Hz perception updates with 2 vehicles (ego + lead)
2. **Compute Control**: Calculate target speed based on lead distance
3. **Send Commands**: Publish control commands back to middleware
4. **Closed Loop**: Ego vehicle speed changes according to ACC logic

### Example Output

**Middleware (simple_demo):**
```
Middleware Engine Initialized
  Simulator:  MockSimulator with 2 vehicles
  IPC:        UDP multicast 239.255.0.1:48198, receive 0.0.0.0:53995

Middleware Running (Press Ctrl+C to stop)
...
^C
Statistics:
  Total cycles:           10000
  Commands received:      9850
  Commands applied:       9850
  Real-time factor:       0.98
```

**ACC Application (test_acc.py):**
```
ACC Application Initialized
  Receiving:  UDP 239.255.0.1:48198 (multicast)
  Sending to: UDP 127.0.0.1:53995
  ACC Config: cruise=20.0 m/s, time_gap=2.0 s

ACC Application Running (Press Ctrl+C to stop)
======================================================================
[   1.0s] Frame  100 | ego_x=   1.50 ego_speed=15.00 target=16.00 | lead_x=  51.00 lead_speed=20.00 dist= 49.50 gap=3.30s
[   2.0s] Frame  200 | ego_x=   3.20 ego_speed=16.00 target=17.00 | lead_x=  55.00 lead_speed=20.00 dist= 51.80 gap=3.24s
[   3.0s] Frame  300 | ego_x=   5.10 ego_speed=17.50 target=18.50 | lead_x=  61.00 lead_speed=20.00 dist= 55.90 gap=3.19s
...
```

## Success Criteria

- ✓ ACC application receives vehicle data at ~100 Hz
- ✓ ACC application sends control commands
- ✓ Middleware receives and applies commands
- ✓ Ego vehicle speed changes based on ACC logic
- ✓ System runs stably without crashes
- ✓ Closed-loop control observable in logs

## Configuration

### Middleware Configuration

In [examples/simple_demo.cpp](../examples/simple_demo.cpp):

```cpp
// MockSimulator config
config.numVehicles = 2;
config.initialEgoSpeed = 15.0;    // Start slower than lead
config.initialLeadSpeed = 20.0;   // Lead at cruise speed
config.leadDistance = 50.0;       // 50m ahead

// Middleware config
engineConfig.timestep = 0.01;     // 100 Hz
engineConfig.enableCommandValidation = true;
engineConfig.enableControlCommands = true;

// UDP IPC config
config.multicastGroup = "239.255.0.1";
config.publishPort = 48198;       // Publish perception
config.receivePort = 53995;       // Receive commands
```

### ACC Configuration

In [apps/test_acc.py](test_acc.py):

```python
self.cruise_speed = 20.0        # Target cruise speed [m/s]
self.time_gap = 2.0              # Desired time gap [s]
self.min_distance = 10.0         # Minimum safe distance [m]
self.max_acceleration = 3.0      # Max accel [m/s²]
self.max_deceleration = 5.0      # Max decel [m/s²]
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│         Middleware (C++)                        │
│  ┌──────────────┐     ┌──────────────┐         │
│  │ MockSimulator├────►│MiddlewareEng │         │
│  └──────────────┘     │              │         │
│                       │   UDP Pub    │◄────┐   │
│  ┌──────────────┐     │  239.255.0.1 │     │   │
│  │ MockSimulator│◄────┤   :48198     │     │   │
│  └──────────────┘     │              │     │   │
│                       │   UDP Recv   │─────┘   │
│                       │  0.0.0.0     │         │
│                       │   :53995     │         │
│                       └──────────────┘         │
└─────────────────────────────────────────────────┘
                              ▲
                              │ Perception Data (VehicleState)
                              │
                              ▼
┌─────────────────────────────────────────────────┐
│         ACC Application (Python)                │
│  ┌──────────────────────────────────────────┐  │
│  │  Receive: UDP 239.255.0.1:48198          │  │
│  │  (multicast join)                        │  │
│  └──────────────┬───────────────────────────┘  │
│                 │                                │
│                 ▼                                │
│  ┌──────────────────────────────────────────┐  │
│  │  ACC Logic                               │  │
│  │  - Calculate following distance          │  │
│  │  - Compute target speed                  │  │
│  │  - Apply time gap control                │  │
│  └──────────────┬───────────────────────────┘  │
│                 │                                │
│                 ▼                                │
│  ┌──────────────────────────────────────────┐  │
│  │  Send: UDP 127.0.0.1:53995               │  │
│  │  (ControlCommand)                        │  │
│  └──────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
                              │
                              │ Control Commands
                              │
                              ▼
                    (Back to Middleware)
```

## Troubleshooting

### No data received in ACC app

```bash
# Check if middleware is publishing
sudo tcpdump -i lo -n udp port 48198

# Check multicast routing
ip maddress show

# Try binding to specific interface
# Modify test_acc.py: bind to '0.0.0.0' instead of ''
```

### Commands not received by middleware

```bash
# Check if commands are being sent
sudo tcpdump -i lo -n udp port 53995

# Verify middleware is listening
netstat -ulnp | grep 53995
```

### Build issues

```bash
# Clean and rebuild
bazel clean
bazel build //examples:simple_demo
```

## Next Steps

Phase 6: Integration testing and performance validation
- Measure end-to-end latency
- Performance benchmarks
- Multi-application testing
- Stress testing

Phase 7+: Real simulator integration (esmini, CarMaker)
