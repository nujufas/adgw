# Simulation Middleware - Implementation Plan

## Overview

**Strategy**: Build and validate the middleware architecture with **simple mock implementations first**, then add real simulator integrations. This de-risks the project by proving the architecture before dealing with simulator complexity.

```
Phase 1-4: Core Middleware (Custom/Mock)
    ↓
Phase 5-6: Validation & Testing
    ↓
Phase 7+: Real Simulator Integration (esmini, CarMaker)
```

---

## Phase 1: Middleware Data Structures (1-2 days)

**Goal**: Define the canonical data format that ALL components will use

**Deliverables**:
- `middleware/data/vehicle_state.h` - Complete VehicleState struct (192 bytes)
- `middleware/data/scene_state.h` - SceneState struct
- `middleware/data/control_command.h` - ControlCommand struct (all modes)
- `middleware/data/sensor_data.h` - SensorData struct (optional for Phase 1)
- Serialization helpers (binary format for IPC)
- Unit tests for all structures

**Tasks**:
```cpp
// Implement and test each structure
middleware::data::VehicleState v;
middleware::data::SceneState s;
middleware::data::ControlCommand c;

// Verify sizes
static_assert(sizeof(VehicleState) == 192);
static_assert(sizeof(ControlCommand) <= 512);

// Test serialization
uint8_t buffer[1024];
size_t size = serialize(v, buffer, sizeof(buffer));
VehicleState v2;
deserialize(buffer, size, v2);
assert(v == v2);
```

**Success Criteria**:
- ✓ All data structures compile and pass unit tests
- ✓ Serialization/deserialization works correctly
- ✓ No dependencies (standalone library)
- ✓ Memory layout verified (alignment, padding)

**Time Estimate**: 1-2 days

---

## Phase 2: Mock Simulator Adapter (1 day)

**Goal**: Create a simple custom simulator for testing - NO esmini/CarMaker yet

**Deliverables**:
- `middleware/simulator/mock_simulator.h/cpp`
- Simple kinematic vehicle model (position, velocity integration)
- Implements full `ISimulator` interface
- Responds to control commands
- Generates synthetic vehicle data

**Implementation**:
```cpp
namespace middleware::simulator {

// Simple mock simulator for testing
class MockSimulator : public ISimulator {
public:
    MockSimulator() {
        // Create 2 vehicles: ego + lead
        vehicles_.resize(2);
        vehicles_[0] = createVehicle(0, 0.0, 0.0, 15.0);   // Ego at origin
        vehicles_[1] = createVehicle(1, 50.0, 0.0, 20.0);  // Lead 50m ahead
    }
    
    bool step(double dt) override {
        // Simple physics: integrate velocity
        for (auto& v : vehicles_) {
            v.pose.x += v.velocity.vx * dt;
            v.pose.y += v.velocity.vy * dt;
            v.velocity.speed = sqrt(v.velocity.vx * v.velocity.vx + 
                                   v.velocity.vy * v.velocity.vy);
        }
        simTime_ += dt;
        return true;
    }
    
    std::vector<middleware::data::VehicleState> getVehicles() override {
        return vehicles_;
    }
    
    bool applyControl(const middleware::data::ControlCommand& cmd) override {
        // Find vehicle and apply control
        for (auto& v : vehicles_) {
            if (v.id == cmd.vehicleId) {
                if (cmd.mode == ControlCommand::Mode::SPEED) {
                    // Set velocity to target speed
                    v.velocity.vx = cmd.speed.targetSpeed;
                    v.velocity.speed = cmd.speed.targetSpeed;
                }
                return true;
            }
        }
        return false;
    }
    
    // ... rest of ISimulator interface
    
private:
    std::vector<middleware::data::VehicleState> vehicles_;
    double simTime_ = 0.0;
};

} // namespace middleware::simulator
```

**Success Criteria**:
- ✓ MockSimulator implements full ISimulator interface
- ✓ Vehicles move according to physics
- ✓ Control commands affect vehicle motion
- ✓ Can run standalone (no middleware yet)

**Time Estimate**: 1 day

---

## Phase 3: UDP IPC Adapter (1-2 days)

**Goal**: Implement bidirectional UDP communication (simplest IPC protocol)

**Deliverables**:
- `middleware/ipc/udp_adapter.h/cpp`
- Publish perception data (VehicleState, SceneState)
- Receive control commands
- Binary serialization over UDP
- Multicast support

**Implementation**:
```cpp
namespace middleware::ipc {

class UDPAdapter : public IIPCAdapter {
public:
    bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) override {
        // Serialize vehicles to binary
        uint8_t buffer[65536];
        size_t size = serializeVehicles(vehicles, buffer, sizeof(buffer));
        
        // Send via UDP
        ssize_t sent = sendto(publishSocket_, buffer, size, 0,
                             (struct sockaddr*)&publishAddr_, sizeof(publishAddr_));
        
        stats_.messagesSent++;
        stats_.bytesSent += sent;
        return sent > 0;
    }
    
    bool receiveCommand(middleware::data::ControlCommand& cmd) override {
        uint8_t buffer[1024];
        sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        
        // Non-blocking receive
        ssize_t received = recvfrom(receiveSocket_, buffer, sizeof(buffer), 
                                    MSG_DONTWAIT, (struct sockaddr*)&from, &fromLen);
        
        if (received > 0) {
            stats_.messagesReceived++;
            stats_.bytesReceived += received;
            return deserializeCommand(buffer, received, cmd);
        }
        return false;
    }
    
    // ... rest of implementation
};

} // namespace middleware::ipc
```

**Testing**:
```bash
# Terminal 1: Middleware publishes
./test_udp_publish

# Terminal 2: Test receiver
./test_udp_receive

# Terminal 3: Test sender (control commands)
./test_udp_send_command
```

**Success Criteria**:
- ✓ Can publish vehicle data via UDP
- ✓ Can receive control commands via UDP
- ✓ Multicast works (multiple receivers)
- ✓ Non-blocking operation
- ✓ Statistics tracking works

**Time Estimate**: 1-2 days

---

## Phase 4: Middleware Engine Core (2 days)

**Goal**: Orchestrate the complete data flow: Simulator → IPC → Simulator

**Deliverables**:
- `middleware/core/middleware_engine.h/cpp`
- Main loop: step simulator, publish data, receive commands, apply commands
- Configuration file support (YAML)
- Logging and monitoring
- Statistics and performance tracking

**Implementation**:
```cpp
namespace middleware::core {

class MiddlewareEngine {
public:
    void run() {
        while (isRunning()) {
            auto t0 = now();
            
            // 1. Step simulator
            simulator_->step(config_.timestep);
            
            // 2. Extract perception data
            auto vehicles = simulator_->getVehicles();
            auto scene = simulator_->getScene();
            
            // 3. Publish to applications
            for (auto& ipc : ipcAdapters_) {
                ipc->publishVehicles(vehicles);
                ipc->publishScene(scene);
            }
            
            // 4. Receive control commands from applications
            middleware::data::ControlCommand cmd;
            for (auto& ipc : ipcAdapters_) {
                if (ipc->receiveCommand(cmd)) {
                    // Validate command
                    if (validateCommand(cmd)) {
                        // Apply to simulator
                        simulator_->applyControl(cmd);
                    }
                }
            }
            
            // 5. Update statistics
            updateStats(now() - t0);
            
            // 6. Rate limiting
            sleepUntil(t0 + config_.timestep);
        }
    }
    
private:
    std::unique_ptr<ISimulator> simulator_;
    std::vector<std::unique_ptr<IIPCAdapter>> ipcAdapters_;
    Config config_;
};

} // namespace middleware::core
```

**Configuration**:
```yaml
# config.yaml
simulator:
  type: mock
  timestep: 0.01  # 100 Hz

middleware:
  update_rate: 100.0
  enable_validation: true

ipc:
  - type: udp
    publish_address: "239.255.0.1"
    publish_port: 48198
    receive_address: "0.0.0.0"
    receive_port: 53995
    multicast: true
```

**Success Criteria**:
- ✓ Main loop runs at configured rate (100 Hz)
- ✓ Data flows: Simulator → IPC → Applications
- ✓ Commands flow: Applications → IPC → Simulator
- ✓ Config file loading works
- ✓ Graceful shutdown on Ctrl+C

**Time Estimate**: 2 days

---

## Phase 5: Custom Test Application (1 day)

**Goal**: Simple ACC application to validate closed-loop control

**Deliverables**:
- `apps/test_acc.py` - Python test application
- Receives vehicle data via UDP
- Computes simple ACC control
- Sends commands back via UDP
- Validates bidirectional communication

**Implementation**:
```python
#!/usr/bin/env python3
"""
Test ACC Application - Validates middleware bidirectional communication
"""

import socket
import struct
import time

class TestACCApp:
    def __init__(self):
        # Receive perception data
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind(('', 48198))
        
        # Join multicast group
        mreq = struct.pack('4sl', socket.inet_aton('239.255.0.1'), socket.INADDR_ANY)
        self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        # Send control commands
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print("Test ACC Application Started")
        print("  Receiving on: UDP 48198 (multicast 239.255.0.1)")
        print("  Sending to:   UDP 53995 (localhost)")
    
    def run(self):
        frame = 0
        while True:
            # 1. Receive perception
            data, _ = self.recv_sock.recvfrom(65536)
            vehicles = self.parse_vehicles(data)
            
            if len(vehicles) < 1:
                continue
            
            ego = vehicles[0]
            frame += 1
            
            # 2. Simple ACC logic
            target_speed = 20.0  # Default cruise speed
            
            if len(vehicles) > 1:
                lead = vehicles[1]
                distance = lead['x'] - ego['x']
                time_gap = distance / max(ego['speed'], 0.1)
                
                if time_gap < 2.0:
                    # Too close - slow down
                    target_speed = max(10.0, ego['speed'] - 1.0)
            
            # 3. Send control command
            cmd = self.create_command(ego['id'], target_speed)
            self.send_sock.sendto(cmd, ('127.0.0.1', 53995))
            
            # 4. Log status
            if frame % 100 == 0:
                print(f"Frame {frame}: ego_speed={ego['speed']:.1f} m/s, "
                      f"target={target_speed:.1f} m/s, distance={distance:.1f} m")
    
    def parse_vehicles(self, data):
        # Parse binary vehicle data (middleware format)
        vehicles = []
        offset = 0
        
        # Simple parser for VehicleState struct
        while offset + 192 <= len(data):
            v = {
                'id': struct.unpack('Q', data[offset:offset+8])[0],
                'timestamp': struct.unpack('d', data[offset+8:offset+16])[0],
                'x': struct.unpack('d', data[offset+16:offset+24])[0],
                'y': struct.unpack('d', data[offset+24:offset+32])[0],
                'speed': struct.unpack('d', data[offset+56:offset+64])[0],
            }
            vehicles.append(v)
            offset += 192
        
        return vehicles
    
    def create_command(self, vehicle_id, target_speed):
        # Create ControlCommand (middleware format)
        cmd = struct.pack('Q', vehicle_id)         # vehicleId
        cmd += struct.pack('d', time.time())       # timestamp
        cmd += struct.pack('Q', 0)                 # sequenceNumber
        cmd += struct.pack('I', 0)                 # mode = SPEED
        cmd += struct.pack('d', target_speed)      # speed.targetSpeed
        return cmd

if __name__ == '__main__':
    app = TestACCApp()
    app.run()
```

**Success Criteria**:
- ✓ Application receives vehicle data
- ✓ Application computes control
- ✓ Application sends commands
- ✓ Ego vehicle responds to commands
- ✓ Closed loop visible (speed changes)

**Time Estimate**: 1 day

---

## Phase 6: Integration Testing & Validation (1 day)

**Goal**: Validate the complete system end-to-end

**Test Scenarios**:

**Test 1: Basic Communication**
```bash
# Start middleware with mock simulator
./sim_middleware config.yaml

# Start test application
python3 apps/test_acc.py

# Expected: Application receives data and sends commands
```

**Test 2: Closed Loop Control**
```
Scenario: Ego vehicle following lead vehicle
- Lead vehicle at 50m, speed 20 m/s
- Ego starts at 0m, speed 15 m/s
- ACC should accelerate ego to match lead speed
- Maintain 40m distance (2 second gap at 20 m/s)

Expected outcome:
- Ego accelerates from 15 → 20 m/s
- Distance stabilizes around 40m
- Ego tracks lead vehicle
```

**Test 3: Multiple Applications**
```bash
# Start middleware
./sim_middleware config.yaml

# Start ACC application
python3 apps/test_acc.py &

# Start visualization
python3 apps/viz.py &

# Start logger
./apps/logger &

# Expected: All applications receive data independently
```

**Test 4: Performance**
```
Measure:
- Update rate (should be 100 Hz)
- Latency (perception → control → actuation)
- CPU usage
- Memory usage

Expected:
- 100 Hz stable
- <10ms latency
- <5% CPU
- <50MB RAM
```

**Deliverables**:
- Test suite (automated)
- Performance benchmarks
- Documentation of results
- Known issues list

**Success Criteria**:
- ✓ All test scenarios pass
- ✓ Closed loop control works
- ✓ Multiple applications supported
- ✓ Performance targets met
- ✓ No crashes or hangs

**Time Estimate**: 1 day

---

## Phase 7: Real Simulator Integration - esmini (2-3 days)

**Goal**: Replace MockSimulator with real esmini integration

**Prerequisites**:
- Phases 1-6 complete and validated
- esmini library installed
- OpenSCENARIO test scenarios

**Deliverables**:
- `middleware/simulator/esmini_adapter.h/cpp`
- Full implementation of all ISimulator methods
- Translation: esmini ↔ middleware format
- OSI ground truth support
- Integration tests with esmini scenarios

**Implementation Details**:
```cpp
namespace middleware::simulator {

class EsminiAdapter : public ISimulator {
public:
    explicit EsminiAdapter(const std::string& scenarioFile) {
        if (SE_Init(scenarioFile.c_str(), 0, 1, 0, 0) != 0) {
            throw std::runtime_error("Failed to initialize esmini");
        }
        egoId_ = 0;
    }
    
    std::vector<middleware::data::VehicleState> getVehicles() override {
        std::vector<middleware::data::VehicleState> vehicles;
        
        int count = SE_GetNumberOfObjects();
        for (int i = 0; i < count; i++) {
            SE_ScenarioObjectState esminiState;
            if (SE_GetObjectState(i, &esminiState) != 0) {
                continue;
            }
            
            // TRANSLATE: esmini → middleware
            middleware::data::VehicleState v{};
            v.id = esminiState.id;
            v.timestamp = esminiState.timestamp;
            v.pose.x = esminiState.x;
            v.pose.y = esminiState.y;
            v.pose.z = esminiState.z;
            // ... complete translation
            
            vehicles.push_back(v);
        }
        
        return vehicles;
    }
    
    bool applyControl(const middleware::data::ControlCommand& cmd) override {
        using Mode = middleware::data::ControlCommand::Mode;
        
        switch (cmd.mode) {
            case Mode::SPEED:
                SE_ReportObjectSpeed(cmd.vehicleId, cmd.speed.targetSpeed);
                return true;
            case Mode::POSITION:
                SE_ReportObjectPos(cmd.vehicleId, cmd.position.x, cmd.position.y, 
                                  cmd.position.z, cmd.position.heading, 
                                  cmd.position.pitch, cmd.position.roll);
                return true;
            default:
                return false;
        }
    }
    
    // ... rest of implementation
};

} // namespace middleware::simulator
```

**Testing**:
```yaml
# config_esmini.yaml
simulator:
  type: esmini
  esmini:
    scenario_file: "scenarios/acc_test.xosc"
    headless: false
    disable_controllers: true
    timestep: 0.01

# Rest same as config.yaml
```

```bash
# Test with esmini
./sim_middleware config_esmini.yaml

# Run same test_acc.py application - should work unchanged!
python3 apps/test_acc.py
```

**Success Criteria**:
- ✓ esmini scenarios load and run
- ✓ Vehicle data extracted correctly
- ✓ Control commands work
- ✓ **Same test application works** (no changes needed)
- ✓ OSI data accessible

**Time Estimate**: 2-3 days

---

## Phase 8: Additional IPC Protocols (2-3 days)

**Goal**: Add TCP, SOME/IP, WebSocket support

**Deliverables**:
- `middleware/ipc/tcp_adapter.h/cpp`
- `middleware/ipc/someip_adapter.h/cpp`
- `middleware/ipc/websocket_adapter.h/cpp`
- Configuration support for multiple IPC adapters
- Web-based visualization tool (WebSocket client)

**Priority Order**:
1. TCP (reliable, connection-oriented)
2. WebSocket (web visualization)
3. SOME/IP (automotive standard, vsomeip integration)

**Time Estimate**: 2-3 days (1 day per protocol)

---

## Phase 9: CarMaker Integration (2-3 days)

**Goal**: Add CarMaker simulator support

**Prerequisites**:
- CarMaker license and installation
- Access to CarMaker API documentation

**Deliverables**:
- `middleware/simulator/carmaker_adapter.h/cpp`
- CarMaker project setup
- TestRun configurations
- Integration tests

**Time Estimate**: 2-3 days

---

## Phase 10: Production Hardening (2-3 days)

**Goal**: Make system production-ready

**Tasks**:
- Error handling and recovery
- Comprehensive logging
- Configuration validation
- Performance optimization
- Documentation (user manual, API docs)
- Deployment scripts
- Docker containerization (optional)

**Time Estimate**: 2-3 days

---

## Summary Timeline

| Phase | Description | Duration | Cumulative |
|-------|-------------|----------|------------|
| 1 | Data Structures | 1-2 days | 2 days |
| 2 | Mock Simulator | 1 day | 3 days |
| 3 | UDP IPC | 1-2 days | 5 days |
| 4 | Middleware Engine | 2 days | 7 days |
| 5 | Test Application | 1 day | 8 days |
| 6 | Integration Testing | 1 day | **9 days** ✓ |
| | **MILESTONE: Working System** | | |
| 7 | esmini Integration | 2-3 days | 12 days |
| 8 | Additional IPC | 2-3 days | 15 days |
| 9 | CarMaker Integration | 2-3 days | 18 days |
| 10 | Production Hardening | 2-3 days | **21 days** |

**Critical Path**: Phases 1-6 (9 days) - Complete working system with mock simulator
**Full System**: Phases 1-10 (21 days) - Production-ready with real simulators

---

## Development Environment Setup

### Required Tools

```bash
# Compiler and build tools
sudo apt install build-essential cmake ninja-build

# Libraries
sudo apt install libyaml-cpp-dev  # Configuration parsing

# Optional: For SOME/IP
sudo apt install vsomeip3-dev

# Optional: For WebSocket
sudo apt install libwebsocketpp-dev

# Python (for test applications)
sudo apt install python3 python3-pip
pip3 install pyyaml numpy
```

### Project Structure

```
sim_middleware/
├── CMakeLists.txt
├── README.md
├── config.yaml                     # Default config
├── config_esmini.yaml             # esmini config
├── config_carmaker.yaml           # CarMaker config
│
├── middleware/
│   ├── data/                      # Phase 1
│   │   ├── vehicle_state.h
│   │   ├── scene_state.h
│   │   ├── control_command.h
│   │   ├── sensor_data.h
│   │   └── serialization.h
│   │
│   ├── simulator/                 # Phase 2, 7, 9
│   │   ├── simulator_interface.h
│   │   ├── mock_simulator.h/cpp
│   │   ├── esmini_adapter.h/cpp
│   │   └── carmaker_adapter.h/cpp
│   │
│   ├── ipc/                       # Phase 3, 8
│   │   ├── ipc_interface.h
│   │   ├── udp_adapter.h/cpp
│   │   ├── tcp_adapter.h/cpp
│   │   ├── someip_adapter.h/cpp
│   │   └── websocket_adapter.h/cpp
│   │
│   ├── core/                      # Phase 4
│   │   ├── middleware_engine.h/cpp
│   │   └── config_parser.h/cpp
│   │
│   └── utils/
│       ├── logger.h
│       └── timer.h
│
├── apps/
│   ├── main.cpp                   # Main middleware executable
│   ├── test_acc.py               # Phase 5: Test application
│   ├── viz.py                    # Visualization
│   └── logger.cpp                # Data logger
│
└── tests/                        # Phase 6
    ├── test_data_structures.cpp
    ├── test_mock_simulator.cpp
    ├── test_udp_adapter.cpp
    ├── test_middleware_engine.cpp
    └── test_integration.cpp
```

---

## Quick Start Guide (After Phase 6)

### 1. Build the middleware

```bash
cd sim_middleware
mkdir build && cd build
cmake .. -GNinja
ninja
```

### 2. Run with mock simulator

```bash
# Terminal 1: Start middleware
./sim_middleware ../config.yaml

# Terminal 2: Start test ACC application
python3 ../apps/test_acc.py
```

### 3. Expected output

```
Terminal 1 (middleware):
  Middleware initialized successfully
  Simulator: MockSimulator
  IPC: UDP (239.255.0.1:48198 → 0.0.0.0:53995)
  Update rate: 100.0 Hz
  Waiting for applications...
  [100 Hz] Frame 0: 2 vehicles, 0 commands
  [100 Hz] Frame 1: 2 vehicles, 1 command received
  [100 Hz] Frame 2: 2 vehicles, 1 command received
  ...

Terminal 2 (application):
  Test ACC Application Started
    Receiving on: UDP 48198 (multicast 239.255.0.1)
    Sending to:   UDP 53995 (localhost)
  Frame 100: ego_speed=15.5 m/s, target=20.0 m/s, distance=48.5 m
  Frame 200: ego_speed=17.2 m/s, target=20.0 m/s, distance=46.8 m
  Frame 300: ego_speed=19.1 m/s, target=20.0 m/s, distance=43.2 m
  Frame 400: ego_speed=20.0 m/s, target=20.0 m/s, distance=40.0 m
  ...
```

---

## Risk Mitigation

### Risk 1: Complex Simulator Integration
**Mitigation**: Start with mock simulator (Phase 2) - validate architecture before real simulators

### Risk 2: IPC Performance
**Mitigation**: Start with simple UDP (Phase 3) - measure and optimize before adding protocols

### Risk 3: Data Format Evolution
**Mitigation**: Version all data structures, use reserved fields for future expansion

### Risk 4: Real-time Requirements
**Mitigation**: Phase 4 includes performance monitoring, Phase 6 validates timing

### Risk 5: Third-party Dependencies
**Mitigation**: Mock simulator has zero dependencies, real simulators added later

---

## Success Metrics

### Phase 6 (Minimum Viable System)
- ✓ 100 Hz update rate stable
- ✓ <10ms end-to-end latency
- ✓ Closed-loop control working
- ✓ Multiple applications supported
- ✓ Zero crashes in 1-hour test

### Phase 10 (Production System)
- ✓ All simulators integrated (mock, esmini, CarMaker)
- ✓ All IPC protocols working (UDP, TCP, SOME/IP, WebSocket)
- ✓ Performance targets met
- ✓ Complete documentation
- ✓ Zero known critical bugs
