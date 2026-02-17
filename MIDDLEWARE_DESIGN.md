# Simulation Middleware - Complete Architecture Design

## Executive Summary

This document describes a **3-tier middleware architecture** that bridges simulators with downstream applications:

```
┌─────────────────────────────────────────────────────────────────────────┐
│  TIER 1: SIMULATOR LAYER (esmini, CarMaker, IPG, etc.)                  │
│  • Vehicle dynamics    • Scene state    • Physics simulation            │
└───────────────────────────┬─────────────────────────────────────────────┘
                            │ C++ API Linking (libesmini.so, libCarMaker.so)
                            │ SE_*, CM_*, etc.
┌───────────────────────────▼─────────────────────────────────────────────┐
│  TIER 2: MIDDLEWARE LAYER (This Application - sim_middleware)           │
│  • Defines canonical data structures (middleware::data::*)              │
│  • Adapts simulator data → middleware format                            │
│  • Exposes IPC interfaces (UDP, TCP, SOME/IP, WebSocket, etc.)         │
│  • Manages data flow and transformation                                 │
└───────────────────────────┬─────────────────────────────────────────────┘
                            │ IPC (UDP, TCP, SOME/IP, WebSocket)
                            │ Middleware data structures ONLY
┌───────────────────────────▼─────────────────────────────────────────────┐
│  TIER 3: APPLICATION LAYER (ACC, Visualization, Logging, etc.)          │
│  • Consumes perception data via IPC                                     │
│  • Produces control commands via IPC                                    │
│  • Uses middleware data format ONLY                                     │
│  • Language agnostic (Python, MATLAB, C++, JavaScript, etc.)            │
└─────────────────────────────────────────────────────────────────────────┘
```

### Key Architectural Principles

1. **Middleware Owns Data Format**: The middleware defines the canonical data structures that ALL applications use
2. **Simulator Adaptation**: Simulators adapt TO the middleware format (bidirectional translation)
3. **IPC-Only Application Interface**: Applications never link to simulator code - only IPC
4. **Multiple IPC Protocols**: UDP, TCP, SOME/IP, WebSocket - applications choose their transport
5. **Pluggable Simulators**: Switch simulators without changing application code
6. **Fully Bidirectional**: Perception flows Simulator→Apps, Control flows Apps→Simulator (closed loop)

## Architecture Overview

### Three-Tier Design

```
╔═══════════════════════════════════════════════════════════════════════════╗
║                         TIER 1: SIMULATOR LAYER                           ║
╚═══════════════════════════════════════════════════════════════════════════╝

    ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
    │   esmini     │      │  CarMaker    │      │   Custom     │
    │   Library    │      │   Library    │      │  Simulator   │
    └──────┬───────┘      └──────┬───────┘      └──────┬───────┘
           │                     │                      │
           │ SE_GetObjectState() │ CM_GetVehicle()     │ GetState()
           │ SE_ReportSpeed()    │ CM_SetSpeed()        │ SetControl()
           │                     │                      │
           └──────────┬──────────┴──────────────────────┘
                      │ C++ Linking (shared libraries)
                      │
╔═══════════════════════▼═══════════════════════════════════════════════════╗
║                    TIER 2: MIDDLEWARE LAYER                               ║
║                    (sim_middleware executable)                            ║
╚═══════════════════════════════════════════════════════════════════════════╝

    ┌─────────────────────────────────────────────────────────────────────┐
    │  SIMULATOR ADAPTERS (BIDIRECTIONAL - C++ API)                       │
    │  ┌────────────┐  ┌────────────┐  ┌────────────┐                   │
    │  │  Esmini    │  │ CarMaker   │  │  Custom    │                   │
    │  │  Adapter   │  │  Adapter   │  │  Adapter   │                   │
    │  │ ↓getState  │  │ ↓getState  │  │ ↓getState  │                   │
    │  │ ↑applyCtrl │  │ ↑applyCtrl │  │ ↑applyCtrl │                   │
    │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘                   │
    │        │↓ ↑             │↓ ↑             │↓ ↑                       │
    │        └────────────────┼────────────────┘                          │
    │                         │↓ Perception                               │
    │                         │↑ Control                                  │
    │                         ▼                                           │
    │         ┌───────────────────────────────────┐                      │
    │         │  MIDDLEWARE DATA STRUCTURES       │                      │
    │         │  (Canonical Format - THE format)  │                      │
    │         │  • VehicleState      ─┐           │                      │
    │         │  • SceneState         │ Perception│                      │
    │         │  • SensorData        ─┘           │                      │
    │         │  • ControlCommand    ─── Control  │                      │
    │         └───────────────┬───────────────────┘                      │
    │                         │↓ ↑                                        │
    │                         ▼                                           │
    │         ┌───────────────────────────────────┐                      │
    │         │  DATA PROCESSING CORE             │                      │
    │         │  • Transformation                 │                      │
    │         │  • Filtering                      │                      │
    │         │  • Validation (both directions)   │                      │
    │         │  • History management             │                      │
    │         │  • Command queuing & safety       │                      │
    │         └───────────────┬───────────────────┘                      │
    │                         │↓ Publish                                  │
    │                         │↑ Receive                                  │
    └─────────────────────────┼───────────────────────────────────────────┘
                              │↓ ↑
    ┌─────────────────────────┼───────────────────────────────────────────┐
    │  IPC ADAPTERS (BIDIRECTIONAL - Network)                             │
    │  ┌────────┐ ┌────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐    │
    │  │  UDP   │ │  TCP   │ │ SOME/IP  │ │WebSocket │ │  Custom  │    │
    │  │ Server │ │ Server │ │  Server  │ │  Server  │ │   IPC    │    │
    │  │↓Pub ↑Rx│ │↓Pub ↑Rx│ │↓Evt ↑Mtd│ │↓Send ↑Rx│ │↓Send ↑Rx│    │
    │  └───┬────┘ └───┬────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘    │
    └──────┼↓↑────────┼↓↑─────────┼↓↑──────────┼↓↑──────────┼↓↑────────┘
           │          │           │            │            │
           │ UDP      │ TCP       │ SOME/IP    │ WebSocket  │ Custom
           │ ↓Pub ↑Rx │ ↓Pub ↑Rx  │ ↓Evt ↑Cmd  │ ↓JSON ↑CMD │ Custom
           │          │           │            │            │
╔══════════▼══════════▼═══════════▼════════════▼════════════▼═══════════════╗
║            ↑              ↑              ↑              ↑              ↑   ║
║                    TIER 3: APPLICATION LAYER                              ║
╚═══════════════════════════════════════════════════════════════════════════╝

    ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐
    │    ACC     │  │Visualization│ │  Logging   │  │  Analysis  │
    │Application │  │    Tool     │  │  Service   │  │    Tool    │
    │ (Python)   │  │ (Web/JS)    │  │   (C++)    │  │ (MATLAB)   │
    │            │  │             │  │            │  │            │
    │ ↓ Receive  │  │ ↓ Receive   │  │ ↓ Receive  │  │ ↓ Receive  │
    │   Perception│  │   Perception│  │   Perception│  │   Perception│
    │            │  │             │  │            │  │            │
    │ ↑ Send     │  │ ↑ Send      │  │            │  │            │
    │   Control  │  │   Commands  │  │ (passive)  │  │ (passive)  │
    └────────────┘  └────────────┘  └────────────┘  └────────────┘
    
    • RECEIVE: VehicleState, SceneState, SensorData (perception) ↓
    • SEND: ControlCommand (actuation) ↑
    • Use middleware data format ONLY
    • Connect via ANY supported IPC protocol (bidirectional)
    • NO simulator library dependencies
    • CLOSED LOOP: Send control, receive feedback
```

### Bidirectional Data Flow

**BIDIRECTIONAL CLOSED LOOP**: Applications send control commands, receive perception feedback

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SIMULATOR                                        │
│                    (esmini / CarMaker / etc.)                            │
└──────┬────────────────────────────────────────────────────────▲─────────┘
       │                                                          │
       │ PERCEPTION (Simulator → Apps)                           │ CONTROL (Apps → Simulator)
       │ • Vehicle positions, velocities                         │ • Speed commands
       │ • Scene geometry, road info                             │ • Acceleration setpoints
       │ • Sensor data                                           │ • Steering inputs
       │                                                          │
┌──────▼──────────────────────────────────────────────────────────────────┐
│                    SIMULATOR ADAPTER                                     │
│  ┌─────────────────────┐              ┌─────────────────────┐          │
│  │  getVehicles()      │              │  applyControl()     │          │
│  │  getScene()         │              │  (receives from     │          │
│  │  getSensors()       │              │   applications)     │          │
│  └──────────┬──────────┘              └──────────▲──────────┘          │
└─────────────┼─────────────────────────────────────┼─────────────────────┘
              │                                      │
              │ Translate: Simulator → Middleware    │ Translate: Middleware → Simulator
              │                                      │
┌─────────────▼──────────────────────────────────────┼─────────────────────┐
│                MIDDLEWARE DATA (CANONICAL)          │                     │
│         VehicleState, SceneState, SensorData        │  ControlCommand     │
│                                                     │                     │
│  ┌──────────────────────────────────────────────────┼──────────────────┐ │
│  │            DATA PROCESSING CORE                  │                  │ │
│  │  • Transform coordinates                         │                  │ │
│  │  • Filter/validate data                          │                  │ │
│  │  • Maintain history                              │                  │ │
│  │  • Command validation & safety checks            │◄─────────────────┘ │
│  └──────────────┬───────────────────────────────────┘                    │
└─────────────────┼──────────────────────────────────────────────────────┬┘
                  │                                                       │
                  │ Serialize & Publish                                  │ Deserialize & Queue
                  │                                                       │
┌─────────────────▼───────────────────────────────────────────────────────▼┐
│                        IPC ADAPTERS (BIDIRECTIONAL)                      │
│  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐│
│  │   UDP Adapter      │  │   TCP Adapter      │  │  SOME/IP Adapter   ││
│  │ • Publish: 48198   │  │ • Publish: 50000   │  │ • Events (0x1234)  ││
│  │ • Receive: 53995   │  │ • Receive: 50000   │  │ • Methods (0x1235) ││
│  └─────────┬──────▲───┘  └─────────┬──────▲───┘  └─────────┬──────▲───┘│
└────────────┼──────┼──────────────── ┼──────┼───────────────┼──────┼────┘
             │      │                 │      │               │      │
             │      │ UDP             │      │ TCP           │      │ SOME/IP
    PUBLISH  │      │ RECEIVE         │      │               │      │
             │      │                 │      │               │      │
┌────────────▼──────┼─────────────────▼──────┼───────────────▼──────┼────┐
│                   │  APPLICATIONS           │                      │    │
│                   │                         │                      │    │
│  ┌────────────────▼──────────┐  ┌──────────▼──────────────┐  ┌───▼────┴─────────┐
│  │  ACC Application           │  │  Visualization Tool     │  │  Analysis Tool   │
│  │  (Python/C++/MATLAB)       │  │  (Web/JavaScript)       │  │  (Python/MATLAB) │
│  │                            │  │                         │  │                  │
│  │  1. Receive perception ◄───┼──┼─┐                       │  │                  │
│  │  2. Compute control        │  │ │                       │  │                  │
│  │  3. Send commands ─────────┼──┼─┼───────────────────────┼──┼──────┐           │
│  └────────────────────────────┘  │ │                       │  │      │           │
│                                   │ └───────────────────────┼──┘      │           │
│                                   └─────────────────────────┘         │           │
└──────────────────────────────────────────────────────────────────────┼───────────┘
                                                                        │
                                            CONTROL COMMANDS            │
                                            (back to middleware)        │
                                                                        │
                                            ┌───────────────────────────┘
                                            │
                                    Loop back to SIMULATOR via IPC
```

**Perception Path (Simulator → Apps)**: Real-time sensor/state data
```
Simulator State (simulator-specific)
    ↓ [Simulator Adapter::getVehicles/getScene/getSensors]
Middleware Data (VehicleState, SceneState, SensorData) - canonical format
    ↓ [Processing Core] transforms/filters/validates
    ↓ [IPC Adapters] serialize & publish via UDP/TCP/SOME-IP
Applications receive perception data
```

**Control Path (Apps → Simulator)**: Commands flow back
```
Application computes control decision (ACC logic, path planning, etc.)
    ↓ [IPC Adapter] sends ControlCommand (middleware format) via UDP/TCP/SOME-IP
Middleware receives & validates command
    ↓ [Simulator Adapter::applyControl] translates to simulator API
Simulator executes control → affects vehicle dynamics
    ↓ (loop back to perception path)
Next perception update reflects control result
```

**This is a CLOSED LOOP SYSTEM**: Control affects simulation, simulation affects perception, perception drives next control decision.

## Middleware Data Structures (Canonical Format)

**Key Point**: This is THE data format. Applications ONLY see these structures.

### vehicle_state.h

```cpp
namespace middleware::data {

// Canonical vehicle state - simulator-independent
struct VehicleState {
    // Identity
    uint64_t id;
    double timestamp;           // seconds since epoch
    
    // Position (global ENU coordinates)
    struct {
        double x, y, z;         // meters
        double heading;         // radians (0 = East, CCW)
        double pitch;           // radians
        double roll;            // radians
    } pose;
    
    // Velocity (global frame)
    struct {
        double vx, vy, vz;      // m/s
        double speed;           // scalar m/s
        double heading_rate;    // rad/s (yaw rate)
    } velocity;
    
    // Acceleration (global frame)
    struct {
        double ax, ay, az;      // m/s²
        double accel;           // longitudinal m/s²
    } acceleration;
    
    // Geometry
    struct {
        double length;          // meters (front to back)
        double width;           // meters (side to side)
        double height;          // meters (ground to top)
    } dimensions;
    
    // Road context
    struct {
        int32_t roadId;         // -1 if off-road
        int32_t laneId;         // -1 if unknown
        double laneOffset;      // meters from lane center (+ = left)
        double sCoord;          // meters along road centerline
    } roadPosition;
    
    // Classification
    enum class Type : uint32_t {
        UNKNOWN = 0,
        CAR = 1,
        TRUCK = 2,
        MOTORCYCLE = 3,
        PEDESTRIAN = 4,
        BICYCLE = 5,
        BUS = 6,
        TRAILER = 7
    } type;
    
    // Status flags (bitfield)
    enum Flags : uint32_t {
        BRAKE_LIGHTS    = 1 << 0,
        LEFT_INDICATOR  = 1 << 1,
        RIGHT_INDICATOR = 1 << 2,
        HAZARD_LIGHTS   = 1 << 3,
        HEADLIGHTS      = 1 << 4,
        REVERSE         = 1 << 5
    };
    uint32_t flags;
    
    // Quality indicators
    float confidence;           // [0, 1] - detection confidence
    uint32_t reserved[4];       // Future use
};

static_assert(sizeof(VehicleState) == 192, "VehicleState size changed");

} // namespace middleware::data
```

### scene_state.h

```cpp
namespace middleware::data {

// Scene state - environmental and road context
struct SceneState {
    double timestamp;
    uint64_t frameNumber;
    
    // Road geometry at reference point (usually ego position)
    struct {
        double curvature;       // 1/m (+ = left curve)
        double slope;           // radians (+ = uphill)
        double speedLimit;      // m/s (0 = unknown)
        int32_t laneCount;
        double roadWidth;       // meters
    } roadGeometry;
    
    // Environmental conditions
    struct {
        double visibility;      // meters
        double precipitation;   // mm/h (rain rate)
        double temperature;     // celsius
        double windSpeed;       // m/s
        double windDirection;   // radians (meteorological)
    } environment;
    
    // Time of day
    struct {
        uint32_t hour;          // [0, 23]
        uint32_t minute;        // [0, 59]
        uint32_t second;        // [0, 59]
        bool isDaytime;
    } timeOfDay;
    
    uint32_t reserved[8];       // Future use
};

} // namespace middleware::data
```

### control_command.h

```cpp
namespace middleware::data {

// Control command - how applications command vehicle behavior
struct ControlCommand {
    uint64_t vehicleId;         // Target vehicle ID
    double timestamp;           // Command timestamp
    uint64_t sequenceNumber;    // Monotonic sequence for ordering
    
    enum class Mode : uint32_t {
        SPEED = 0,              // Direct speed control
        ACCELERATION = 1,       // Acceleration setpoint
        THROTTLE_BRAKE = 2,     // Pedal positions [0, 1]
        TRAJECTORY = 3,         // Path following with waypoints
        POSITION = 4            // Teleport to position (debug only)
    } mode;
    
    // Union of different control modes
    union {
        // SPEED mode
        struct {
            double targetSpeed;  // m/s
            double tolerance;    // m/s (acceptable deviation)
        } speed;
        
        // ACCELERATION mode
        struct {
            double targetAccel;  // m/s²
            double maxJerk;      // m/s³ (rate limit)
        } acceleration;
        
        // THROTTLE_BRAKE mode
        struct {
            float throttle;      // [0, 1]
            float brake;         // [0, 1]
            float steer;         // [-1, 1] (left negative)
        } pedals;
        
        // TRAJECTORY mode
        struct {
            double waypoints[32][2];  // x, y pairs (max 32 waypoints)
            uint32_t waypointCount;
            double targetSpeed;       // m/s along trajectory
            double lookaheadTime;     // seconds
        } trajectory;
        
        // POSITION mode (debug/testing)
        struct {
            double x, y, z;
            double heading, pitch, roll;
        } position;
    };
    
    // Safety limits
    struct {
        double maxSpeed;        // m/s
        double maxAccel;        // m/s²
        double maxDecel;        // m/s²
    } limits;
    
    // Metadata
    uint32_t priority;          // Higher = more important
    uint32_t reserved[4];
};

static_assert(sizeof(ControlCommand) <= 512, "ControlCommand too large");

} // namespace middleware::data
```

### sensor_data.h

```cpp
namespace middleware::data {

// Sensor data - raw perception (optional, for advanced applications)
struct SensorData {
    uint64_t vehicleId;
    double timestamp;
    
    // Lidar point cloud
    struct {
        float* points;          // x,y,z,intensity interleaved
        size_t pointCount;
        uint32_t sensorId;
    } lidar;
    
    // Camera image
    struct {
        uint8_t* data;
        uint32_t width, height;
        enum Format : uint32_t {
            RGB8 = 0,
            RGBA8 = 1,
            BGR8 = 2,
            GRAY8 = 3
        } format;
        uint32_t sensorId;
    } camera;
    
    // Radar detections
    struct RadarTarget {
        double range;           // meters
        double azimuth;         // radians
        double elevation;       // radians
        double rangeRate;       // m/s (doppler)
        float rcs;              // radar cross section (dBsm)
    };
    RadarTarget* radarTargets;
    size_t radarCount;
    
    uint32_t reserved[4];
};

} // namespace middleware::data
```

## Simulator Adapters (Tier 1 ↔ Tier 2)

**Purpose**: BIDIRECTIONAL translation between simulator and middleware

**Key Point**: 
- **OUTBOUND** (Simulator → Middleware): Extract perception data (getVehicles, getScene, getSensors)
- **INBOUND** (Middleware → Simulator): Apply control commands (applyControl)
- Applications never see simulator APIs - only middleware format via IPC

### simulator_interface.h

```cpp
namespace middleware::simulator {

// Abstract simulator interface
class ISimulator {
public:
    virtual ~ISimulator() = default;
    
    // Lifecycle
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    virtual bool step(double dt) = 0;
    virtual bool isRunning() const = 0;
    
    // OUTBOUND: Simulator → Middleware (perception)
    virtual std::vector<middleware::data::VehicleState> getVehicles() = 0;
    virtual middleware::data::SceneState getScene() = 0;
    virtual middleware::data::SensorData getSensors(uint64_t vehicleId) = 0;
    
    // INBOUND: Middleware → Simulator (control)
    virtual bool applyControl(const middleware::data::ControlCommand& cmd) = 0;
    
    // Metadata
    virtual const char* getName() const = 0;
    virtual const char* getVersion() const = 0;
    virtual uint64_t getEgoVehicleId() const = 0;
    virtual double getSimulationTime() const = 0;
};

// Factory for creating simulator instances
class SimulatorFactory {
public:
    enum class Type {
        ESMINI,
        CARMAKER,
        CUSTOM
    };
    
    static std::unique_ptr<ISimulator> create(Type type, const std::string& config);
    static std::unique_ptr<ISimulator> createFromConfig(const std::string& configFile);
};

} // namespace middleware::simulator
```

### esmini_adapter.h

```cpp
namespace middleware::simulator {

// esmini simulator adapter
class EsminiAdapter : public ISimulator {
public:
    struct Config {
        std::string scenarioFile;
        bool headless;
        bool disableControllers;
        double timestep;
    };
    
    explicit EsminiAdapter(const Config& config);
    ~EsminiAdapter() override;
    
    // Lifecycle
    bool initialize() override;
    void shutdown() override;
    bool step(double dt) override;
    bool isRunning() const override;
    
    // OUTBOUND: esmini → middleware format
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
            
            // Position
            v.pose.x = esminiState.x;
            v.pose.y = esminiState.y;
            v.pose.z = esminiState.z;
            v.pose.heading = esminiState.h;
            v.pose.pitch = esminiState.p;
            v.pose.roll = esminiState.r;
            
            // Velocity
            v.velocity.vx = esminiState.speed * cos(esminiState.h);
            v.velocity.vy = esminiState.speed * sin(esminiState.h);
            v.velocity.vz = 0.0;
            v.velocity.speed = esminiState.speed;
            v.velocity.heading_rate = 0.0;  // esmini doesn't provide this
            
            // Acceleration (compute from history if available)
            v.acceleration.ax = 0.0;
            v.acceleration.ay = 0.0;
            v.acceleration.az = 0.0;
            v.acceleration.accel = 0.0;
            
            // Dimensions
            v.dimensions.length = esminiState.boundingbox.dimensions.length_;
            v.dimensions.width = esminiState.boundingbox.dimensions.width_;
            v.dimensions.height = esminiState.boundingbox.dimensions.height_;
            
            // Road position
            v.roadPosition.roadId = esminiState.roadId;
            v.roadPosition.laneId = esminiState.laneId;
            v.roadPosition.laneOffset = esminiState.laneOffset;
            v.roadPosition.sCoord = esminiState.s;
            
            // Type (map esmini type to middleware type)
            v.type = mapVehicleType(esminiState.model_id);
            v.flags = 0;
            v.confidence = 1.0;  // Perfect sensor
            
            vehicles.push_back(v);
        }
        
        return vehicles;
    }
    
    middleware::data::SceneState getScene() override {
        middleware::data::SceneState scene{};
        scene.timestamp = SE_GetSimulationTime();
        scene.frameNumber = frameNumber_++;
        
        // Get road geometry at ego position
        SE_RoadInfo roadInfo;
        if (SE_GetRoadInfoAtDistance(egoId_, 0.0, &roadInfo, 0) == 0) {
            scene.roadGeometry.curvature = roadInfo.local_curvature;
            scene.roadGeometry.slope = roadInfo.angle;
            scene.roadGeometry.speedLimit = roadInfo.speed_limit;
        }
        
        // Environment (esmini doesn't provide this - use defaults)
        scene.environment.visibility = 1000.0;
        scene.environment.precipitation = 0.0;
        scene.environment.temperature = 20.0;
        
        return scene;
    }
    
    middleware::data::SensorData getSensors(uint64_t vehicleId) override {
        middleware::data::SensorData sensors{};
        sensors.vehicleId = vehicleId;
        sensors.timestamp = SE_GetSimulationTime();
        
        // esmini provides OSI ground truth - translate to middleware format
        int size;
        const char* osi_data = SE_GetOSIGroundTruthRaw(&size);
        if (osi_data) {
            translateOSIToMiddleware(osi_data, size, sensors);
        }
        
        return sensors;
    }
    
    // INBOUND: middleware → esmini
    bool applyControl(const middleware::data::ControlCommand& cmd) override {
        using Mode = middleware::data::ControlCommand::Mode;
        
        switch (cmd.mode) {
            case Mode::SPEED:
                SE_ReportObjectSpeed(cmd.vehicleId, cmd.speed.targetSpeed);
                return true;
                
            case Mode::ACCELERATION: {
                // esmini doesn't have direct acceleration control
                // Compute target speed from current speed + accel * dt
                double currentSpeed = getCurrentSpeed(cmd.vehicleId);
                double targetSpeed = currentSpeed + cmd.acceleration.targetAccel * timestep_;
                SE_ReportObjectSpeed(cmd.vehicleId, targetSpeed);
                return true;
            }
            
            case Mode::POSITION:
                SE_ReportObjectPos(cmd.vehicleId, 
                                  cmd.position.x, cmd.position.y, cmd.position.z,
                                  cmd.position.heading, cmd.position.pitch, cmd.position.roll);
                return true;
                
            default:
                return false;
        }
    }
    
    // Metadata
    const char* getName() const override { return "esmini"; }
    const char* getVersion() const override { return SE_GetVersion(); }
    uint64_t getEgoVehicleId() const override { return egoId_; }
    double getSimulationTime() const override { return SE_GetSimulationTime(); }
    
private:
    Config config_;
    uint64_t egoId_;
    uint64_t frameNumber_;
    double timestep_;
    
    middleware::data::VehicleState::Type mapVehicleType(int esminiType);
    void translateOSIToMiddleware(const char* osi, int size, middleware::data::SensorData& sensors);
    double getCurrentSpeed(uint64_t vehicleId);
};

} // namespace middleware::simulator
```

### carmaker_adapter.h

```cpp
namespace middleware::simulator {

// CarMaker simulator adapter
class CarMakerAdapter : public ISimulator {
public:
    struct Config {
        std::string projectPath;
        std::string testrunName;
        std::string vehicleModel;
        bool useIPG;            // Use IPG communication
        uint16_t ipgPort;
    };
    
    explicit CarMakerAdapter(const Config& config);
    ~CarMakerAdapter() override;
    
    bool initialize() override;
    void shutdown() override;
    bool step(double dt) override;
    bool isRunning() const override;
    
    // OUTBOUND: CarMaker → middleware format
    std::vector<middleware::data::VehicleState> getVehicles() override {
        std::vector<middleware::data::VehicleState> vehicles;
        
        // Get ego vehicle from CarMaker
        tCMVehicle* cmVeh = CMGetVehicle();
        if (!cmVeh) return vehicles;
        
        // TRANSLATE: CarMaker → middleware
        middleware::data::VehicleState v{};
        v.id = 0;  // Ego vehicle
        v.timestamp = CMGetTime();
        
        // Position (CarMaker uses different coordinate system)
        v.pose.x = cmVeh->Fr1.t_0[0];
        v.pose.y = cmVeh->Fr1.t_0[1];
        v.pose.z = cmVeh->Fr1.t_0[2];
        
        // Orientation (convert CarMaker Euler angles)
        v.pose.heading = cmVeh->Fr1.yaw;
        v.pose.pitch = cmVeh->Fr1.pitch;
        v.pose.roll = cmVeh->Fr1.roll;
        
        // Velocity
        v.velocity.vx = cmVeh->v[0];
        v.velocity.vy = cmVeh->v[1];
        v.velocity.vz = cmVeh->v[2];
        v.velocity.speed = sqrt(v.velocity.vx * v.velocity.vx + 
                               v.velocity.vy * v.velocity.vy);
        v.velocity.heading_rate = cmVeh->omega[2];
        
        // Acceleration
        v.acceleration.ax = cmVeh->a[0];
        v.acceleration.ay = cmVeh->a[1];
        v.acceleration.az = cmVeh->a[2];
        v.acceleration.accel = cmVeh->a[0];  // Longitudinal
        
        // Dimensions
        v.dimensions.length = cmVeh->length;
        v.dimensions.width = cmVeh->width;
        v.dimensions.height = cmVeh->height;
        
        // Road position (from CarMaker Road module)
        tCMRoad* road = CMGetRoad();
        if (road) {
            v.roadPosition.roadId = road->roadId;
            v.roadPosition.laneId = road->laneId;
            v.roadPosition.laneOffset = road->lateralOffset;
            v.roadPosition.sCoord = road->sCoord;
        }
        
        v.type = middleware::data::VehicleState::Type::CAR;
        v.confidence = 1.0;
        
        vehicles.push_back(v);
        
        // Get traffic vehicles if available
        addTrafficVehicles(vehicles);
        
        return vehicles;
    }
    
    middleware::data::SceneState getScene() override;
    middleware::data::SensorData getSensors(uint64_t vehicleId) override;
    
    // INBOUND: middleware → CarMaker
    bool applyControl(const middleware::data::ControlCommand& cmd) override {
        using Mode = middleware::data::ControlCommand::Mode;
        
        switch (cmd.mode) {
            case Mode::SPEED:
                CMSetSpeed(cmd.speed.targetSpeed);
                return true;
                
            case Mode::ACCELERATION:
                CMSetAccel(cmd.acceleration.targetAccel);
                return true;
                
            case Mode::THROTTLE_BRAKE:
                CMSetThrottle(cmd.pedals.throttle);
                CMSetBrake(cmd.pedals.brake);
                CMSetSteer(cmd.pedals.steer);
                return true;
                
            default:
                return false;
        }
    }
    
    const char* getName() const override { return "CarMaker"; }
    const char* getVersion() const override { return CMGetVersion(); }
    uint64_t getEgoVehicleId() const override { return 0; }
    double getSimulationTime() const override { return CMGetTime(); }
    
private:
    Config config_;
    void* cmHandle_;
    void* ipgHandle_;
    
    void addTrafficVehicles(std::vector<middleware::data::VehicleState>& vehicles);
};

} // namespace middleware::simulator
```

## IPC Adapters (Tier 2 → Tier 3)

**Purpose**: Expose middleware data to applications via network protocols

**Key Point**: Applications connect via IPC, never link to simulator libraries

### ipc_interface.h

```cpp
namespace middleware::ipc {

// Abstract IPC adapter interface
class IIPCAdapter {
public:
    virtual ~IIPCAdapter() = default;
    
    // Lifecycle
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
    
    // PUBLISH: Middleware → Applications (perception)
    virtual bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) = 0;
    virtual bool publishScene(const middleware::data::SceneState& scene) = 0;
    virtual bool publishSensors(const middleware::data::SensorData& sensors) = 0;
    
    // RECEIVE: Applications → Middleware (control)
    virtual bool receiveCommand(middleware::data::ControlCommand& cmd) = 0;
    virtual bool hasCommand() const = 0;
    virtual size_t receiveCommands(std::vector<middleware::data::ControlCommand>& cmds, size_t maxCount) = 0;
    
    // Status
    virtual bool isConnected() const = 0;
    virtual size_t clientCount() const = 0;
    virtual const char* getName() const = 0;
    
    // Statistics
    struct Stats {
        uint64_t messagesSent;
        uint64_t messagesReceived;
        uint64_t bytesSent;
        uint64_t bytesReceived;
        double avgLatency;      // ms
    };
    virtual Stats getStats() const = 0;
};

} // namespace middleware::ipc
```

### udp_adapter.h

```cpp
namespace middleware::ipc {

// UDP adapter - simple multicast/unicast
class UDPAdapter : public IIPCAdapter {
public:
    struct Config {
        // Publishing (perception data out)
        std::string publishAddress;   // e.g., "239.255.0.1" (multicast) or "127.0.0.1"
        uint16_t publishPort;         // e.g., 48198
        bool multicast;
        
        // Receiving (control commands in)
        std::string receiveAddress;   // e.g., "0.0.0.0" (any)
        uint16_t receivePort;         // e.g., 53995
        
        // Options
        size_t bufferSize;
        int32_t ttl;                  // multicast TTL
    };
    
    explicit UDPAdapter(const Config& config);
    ~UDPAdapter() override;
    
    bool initialize() override;
    void shutdown() override;
    
    bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) override;
    bool publishScene(const middleware::data::SceneState& scene) override;
    bool publishSensors(const middleware::data::SensorData& sensors) override;
    
    bool receiveCommand(middleware::data::ControlCommand& cmd) override;
    bool hasCommand() const override;
    size_t receiveCommands(std::vector<middleware::data::ControlCommand>& cmds, size_t maxCount) override;
    
    bool isConnected() const override { return publishSocket_ >= 0; }
    size_t clientCount() const override { return 0; }  // UDP is connectionless
    const char* getName() const override { return "UDP"; }
    Stats getStats() const override { return stats_; }
    
private:
    Config config_;
    int publishSocket_;
    int receiveSocket_;
    Stats stats_;
    
    // Serialization
    size_t serializeVehicles(const std::vector<middleware::data::VehicleState>& vehicles, 
                            uint8_t* buffer, size_t bufferSize);
    size_t serializeScene(const middleware::data::SceneState& scene,
                         uint8_t* buffer, size_t bufferSize);
    bool deserializeCommand(const uint8_t* buffer, size_t size,
                           middleware::data::ControlCommand& cmd);
};

} // namespace middleware::ipc
```

### tcp_adapter.h

```cpp
namespace middleware::ipc {

// TCP adapter - connection-oriented, reliable
class TCPAdapter : public IIPCAdapter {
public:
    struct Config {
        uint16_t port;
        size_t maxClients;
        size_t bufferSize;
        bool blocking;
        uint32_t keepaliveInterval;  // seconds
    };
    
    explicit TCPAdapter(const Config& config);
    ~TCPAdapter() override;
    
    bool initialize() override;
    void shutdown() override;
    
    bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) override;
    bool publishScene(const middleware::data::SceneState& scene) override;
    bool publishSensors(const middleware::data::SensorData& sensors) override;
    
    bool receiveCommand(middleware::data::ControlCommand& cmd) override;
    bool hasCommand() const override;
    size_t receiveCommands(std::vector<middleware::data::ControlCommand>& cmds, size_t maxCount) override;
    
    bool isConnected() const override;
    size_t clientCount() const override { return clients_.size(); }
    const char* getName() const override { return "TCP"; }
    Stats getStats() const override { return stats_; }
    
private:
    Config config_;
    int serverSocket_;
    
    struct Client {
        int socket;
        std::string address;
        uint16_t port;
        double lastActivity;
    };
    std::vector<Client> clients_;
    
    Stats stats_;
    
    void acceptClients();
    void removeDeadClients();
    bool sendToClient(Client& client, const uint8_t* data, size_t size);
    size_t receiveFromClients(uint8_t* buffer, size_t bufferSize);
};

} // namespace middleware::ipc
```

### someip_adapter.h

```cpp
namespace middleware::ipc {

// SOME/IP adapter - automotive standard (using vsomeip)
class SOMEIPAdapter : public IIPCAdapter {
public:
    struct Config {
        std::string applicationName;
        uint16_t serviceId;
        uint16_t instanceId;
        std::string configFile;  // vsomeip JSON config
    };
    
    explicit SOMEIPAdapter(const Config& config);
    ~SOMEIPAdapter() override;
    
    bool initialize() override;
    void shutdown() override;
    
    // Publish as SOME/IP events
    bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) override;
    bool publishScene(const middleware::data::SceneState& scene) override;
    bool publishSensors(const middleware::data::SensorData& sensors) override;
    
    // Receive via SOME/IP methods
    bool receiveCommand(middleware::data::ControlCommand& cmd) override;
    bool hasCommand() const override;
    size_t receiveCommands(std::vector<middleware::data::ControlCommand>& cmds, size_t maxCount) override;
    
    bool isConnected() const override;
    size_t clientCount() const override;
    const char* getName() const override { return "SOME/IP"; }
    Stats getStats() const override { return stats_; }
    
private:
    Config config_;
    std::shared_ptr<vsomeip::application> app_;
    Stats stats_;
    
    std::queue<middleware::data::ControlCommand> commandQueue_;
    std::mutex queueMutex_;
    
    // vsomeip callbacks
    void onMessage(const std::shared_ptr<vsomeip::message>& msg);
    void onAvailability(vsomeip::service_t service, vsomeip::instance_t instance, bool available);
};

} // namespace middleware::ipc
```

### websocket_adapter.h

```cpp
namespace middleware::ipc {

// WebSocket adapter - for web-based visualization tools
class WebSocketAdapter : public IIPCAdapter {
public:
    struct Config {
        uint16_t port;
        std::string webRoot;       // Serve static files (HTML/JS)
        bool enableHTTP;           // Also serve HTTP
        size_t maxClients;
        std::string corsOrigin;    // CORS policy
    };
    
    explicit WebSocketAdapter(const Config& config);
    ~WebSocketAdapter() override;
    
    bool initialize() override;
    void shutdown() override;
    
    // Publish as JSON messages
    bool publishVehicles(const std::vector<middleware::data::VehicleState>& vehicles) override;
    bool publishScene(const middleware::data::SceneState& scene) override;
    bool publishSensors(const middleware::data::SensorData& sensors) override;
    
    // Receive JSON commands
    bool receiveCommand(middleware::data::ControlCommand& cmd) override;
    bool hasCommand() const override;
    size_t receiveCommands(std::vector<middleware::data::ControlCommand>& cmds, size_t maxCount) override;
    
    bool isConnected() const override;
    size_t clientCount() const override;
    const char* getName() const override { return "WebSocket"; }
    Stats getStats() const override { return stats_; }
    
private:
    Config config_;
    void* wsServer_;  // websocketpp server
    Stats stats_;
    
    std::queue<middleware::data::ControlCommand> commandQueue_;
    std::mutex queueMutex_;
    
    // JSON serialization
    std::string toJSON(const std::vector<middleware::data::VehicleState>& vehicles);
    std::string toJSON(const middleware::data::SceneState& scene);
    bool fromJSON(const std::string& json, middleware::data::ControlCommand& cmd);
};

} // namespace middleware::ipc
```

## Middleware Core

### middleware_engine.h

```cpp
namespace middleware::core {

// Main middleware engine - orchestrates everything
class MiddlewareEngine {
public:
    struct Config {
        // Simulator config
        middleware::simulator::SimulatorFactory::Type simulatorType;
        std::string simulatorConfig;
        
        // IPC configs
        std::vector<std::unique_ptr<middleware::ipc::IIPCAdapter>> ipcAdapters;
        
        // Processing options
        double updateRate;         // Hz (e.g., 100 Hz)
        size_t historySize;        // Frames to keep
        bool enableValidation;
        bool enableRecording;
        std::string recordingPath;
    };
    
    explicit MiddlewareEngine(const Config& config);
    ~MiddlewareEngine();
    
    // Lifecycle
    bool initialize();
    void shutdown();
    
    // Main loop
    void run();           // Run until simulator stops
    void step();          // Single step
    bool isRunning() const;
    
    // Status
    struct Status {
        double simulationTime;
        uint64_t frameNumber;
        size_t vehicleCount;
        size_t ipcClientCount;
        double updateRate;
        double avgProcessingTime;
    };
    Status getStatus() const;
    
private:
    Config config_;
    std::unique_ptr<middleware::simulator::ISimulator> simulator_;
    std::vector<std::unique_ptr<middleware::ipc::IIPCAdapter>> ipcAdapters_;
    
    // Data pipeline
    void processPerception();
    void processControl();
    void publishData();
    
    // History management
    struct FrameData {
        std::vector<middleware::data::VehicleState> vehicles;
        middleware::data::SceneState scene;
        double timestamp;
    };
    std::deque<FrameData> history_;
    
    // Statistics
    Status status_;
    std::chrono::steady_clock::time_point lastUpdate_;
};

} // namespace middleware::core
```

## Main Application

### main.cpp

```cpp
#include "middleware_engine.h"
#include "config_parser.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: sim_middleware <config.yaml>" << std::endl;
        return 1;
    }
    
    // Parse configuration
    auto config = middleware::config::parse(argv[1]);
    if (!config) {
        std::cerr << "Failed to parse config file" << std::endl;
        return 1;
    }
    
    // Create middleware engine
    middleware::core::MiddlewareEngine engine(*config);
    
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize middleware" << std::endl;
        return 1;
    }
    
    std::cout << "Middleware initialized successfully" << std::endl;
    std::cout << "Waiting for applications to connect..." << std::endl;
    
    // Run main loop
    engine.run();
    
    // Cleanup
    engine.shutdown();
    std::cout << "Middleware shutdown complete" << std::endl;
    
    return 0;
}
```

### config.yaml

```yaml
# Simulation Middleware Configuration

simulator:
  type: esmini  # Options: esmini, carmaker
  
  # esmini-specific
  esmini:
    scenario_file: "scenarios/acc_scenario.xosc"
    headless: false
    disable_controllers: true
    timestep: 0.01
  
  # CarMaker-specific (not used if type=esmini)
  carmaker:
    project_path: "/path/to/carmaker/project"
    testrun: "ACC_Test"
    vehicle_model: "Sedan_Medium"

middleware:
  update_rate: 100.0  # Hz
  history_size: 100   # frames
  enable_validation: true
  enable_recording: false
  recording_path: "data/recording.bin"

ipc:
  # UDP adapter
  - type: udp
    publish_address: "239.255.0.1"
    publish_port: 48198
    multicast: true
    receive_address: "0.0.0.0"
    receive_port: 53995
    buffer_size: 65536
  
  # TCP adapter
  - type: tcp
    port: 50000
    max_clients: 10
    buffer_size: 65536
    blocking: false
  
  # SOME/IP adapter
  - type: someip
    application_name: "sim_middleware"
    service_id: 0x1234
    instance_id: 0x5678
    config_file: "vsomeip_config.json"
  
  # WebSocket adapter
  - type: websocket
    port: 8080
    web_root: "web/"
    enable_http: true
    max_clients: 20
```

## Application Integration

### Example: ACC Application (Python via UDP) - BIDIRECTIONAL

```python
#!/usr/bin/env python3
"""
ACC application - BIDIRECTIONAL communication with middleware via UDP
- RECEIVES perception data (vehicles, scene) on port 48198
- SENDS control commands (speed setpoints) to port 53995
"""

import socket
import struct
import time

# Middleware data structures (must match middleware::data)
class VehicleState:
    def __init__(self, data):
        # Parse binary data from middleware
        # Format matches middleware::data::VehicleState
        self.id = struct.unpack('Q', data[0:8])[0]
        self.timestamp = struct.unpack('d', data[8:16])[0]
        self.x = struct.unpack('d', data[16:24])[0]
        self.y = struct.unpack('d', data[24:32])[0]
        # ... parse rest of structure

class ControlCommand:
    def __init__(self, vehicle_id, target_speed):
        self.vehicle_id = vehicle_id
        self.timestamp = time.time()
        self.mode = 0  # SPEED mode
        self.target_speed = target_speed
    
    def serialize(self):
        # Pack to binary format matching middleware::data::ControlCommand
        data = struct.pack('Q', self.vehicle_id)  # vehicleId
        data += struct.pack('d', self.timestamp)   # timestamp
        data += struct.pack('I', self.mode)        # mode
        data += struct.pack('d', self.target_speed) # speed.targetSpeed
        return data

def main():
    # BIDIRECTIONAL UDP setup
    
    # RECEIVE socket: Listen for perception data from middleware
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(('', 48198))  # Middleware publishes on 48198
    
    # Join multicast group (if middleware uses multicast)
    mreq = struct.pack('4sl', socket.inet_aton('239.255.0.1'), socket.INADDR_ANY)
    recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    # SEND socket: Send control commands to middleware
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("ACC application connected to middleware (BIDIRECTIONAL)")
    print("  ↓ Receiving perception data on port 48198")
    print("  ↑ Sending control commands to port 53995")
    print("")
    
    frame_count = 0
    
    while True:
        # ============================================================
        # STEP 1: RECEIVE perception data from middleware
        # ============================================================
        data, addr = recv_sock.recvfrom(65536)
        
        # Parse vehicle states (middleware format)
        vehicles = parse_vehicles(data)
        
        if not vehicles:
            continue
        
        ego = vehicles[0]
        frame_count += 1
        
        # ============================================================
        # STEP 2: COMPUTE control decision (ACC algorithm)
        # ============================================================
        target_speed = 20.0  # m/s (default cruise speed)
        
        # Find lead vehicle in same lane
        lead = find_lead_vehicle(ego, vehicles[1:])
        if lead:
            # Following mode - maintain safe distance
            distance = lead.x - ego.x
            time_gap = distance / max(ego.speed, 0.1)
            if time_gap < 2.0:
                # Too close - slow down
                target_speed = min(ego.speed - 2.0, lead.speed)
        
        # ============================================================
        # STEP 3: SEND control command back to middleware
        # ============================================================
        cmd = ControlCommand(ego.id, target_speed)
        send_sock.sendto(cmd.serialize(), ('127.0.0.1', 53995))
        
        # ============================================================
        # STEP 4: Next iteration - receive updated perception
        #         (which reflects our control command)
        # ============================================================
        if frame_count % 100 == 0:
            print(f"Frame {frame_count}: ego_speed={ego.speed:.1f} m/s, "
                  f"target={target_speed:.1f} m/s, lead={'Yes' if lead else 'No'}")

if __name__ == '__main__':
    main()
```

## Project Structure

```
sim_middleware/
├── CMakeLists.txt
├── README.md
├── config.yaml                      # Main configuration
│
├── middleware/                      # Core middleware code
│   ├── data/                        # Data structures (CANONICAL FORMAT)
│   │   ├── vehicle_state.h
│   │   ├── scene_state.h
│   │   ├── control_command.h
│   │   └── sensor_data.h
│   │
│   ├── simulator/                   # Simulator adapters (inbound)
│   │   ├── simulator_interface.h
│   │   ├── simulator_factory.cpp
│   │   ├── esmini_adapter.h
│   │   ├── esmini_adapter.cpp
│   │   ├── carmaker_adapter.h
│   │   └── carmaker_adapter.cpp
│   │
│   ├── ipc/                         # IPC adapters (outbound)
│   │   ├── ipc_interface.h
│   │   ├── udp_adapter.h
│   │   ├── udp_adapter.cpp
│   │   ├── tcp_adapter.h
│   │   ├── tcp_adapter.cpp
│   │   ├── someip_adapter.h
│   │   ├── someip_adapter.cpp
│   │   ├── websocket_adapter.h
│   │   └── websocket_adapter.cpp
│   │
│   ├── core/                        # Middleware engine
│   │   ├── middleware_engine.h
│   │   ├── middleware_engine.cpp
│   │   ├── data_processor.h
│   │   └── data_processor.cpp
│   │
│   └── utils/                       # Utilities
│       ├── config_parser.h
│       ├── serialization.h
│       └── logger.h
│
├── apps/                            # Application examples
│   ├── main.cpp                     # Main middleware executable
│   ├── acc_app.py                   # Python ACC application
│   ├── visualization.html           # Web visualization (WebSocket)
│   └── logger_app.cpp               # C++ data logger
│
└── tests/                           # Unit tests
    ├── test_data_structures.cpp
    ├── test_simulator_adapters.cpp
    └── test_ipc_adapters.cpp
```

## Key Advantages of This Architecture

### 1. **Fully Bidirectional Communication**
- **Perception Flow**: Simulator → Middleware → Applications
- **Control Flow**: Applications → Middleware → Simulator
- **Closed Loop**: Control affects simulation, simulation affects next perception
- Real-time feedback loop at 100+ Hz

### 2. **Clear Separation of Concerns**
- **Simulator Layer**: Physics, dynamics (esmini, CarMaker)
- **Middleware Layer**: Data translation (both ways), IPC management, validation
- **Application Layer**: Business logic (ACC, viz, logging)

### 3. **Simulator Independence**
- Applications use middleware format ONLY
- Switch simulators without touching application code
- Add new simulators by implementing `ISimulator` (bidirectional interface)

### 4. **Multiple IPC Options (All Bidirectional)**
- UDP: Fast, multicast capable, pub/sub pattern
- TCP: Reliable, connection-oriented, request/response
- SOME/IP: Automotive standard, events + methods
- WebSocket: Web-based tools, JSON messaging

### 5. **Language Agnostic**
- Python, MATLAB, JavaScript, C++
- Any language with socket/IPC support
- No simulator library dependencies
- Same bidirectional protocol for all

### 6. **Scalable & Distributed**
- Run applications on different machines
- Multiple applications simultaneously (many-to-one control supported)
- One middleware, many consumers AND producers

### 7. **Production Ready**
- Uses automotive standards (SOME/IP)
- Supports CarMaker (industry standard)
- Real-time capable with closed-loop control
- Comprehensive error handling and command validation

## Implementation Phases

### Overview

**Strategy**: Build and validate the middleware architecture with **simple mock implementations first**, then add real simulator integrations. This de-risks the project by proving the architecture before dealing with simulator complexity.

```
Phase 1-4: Core Middleware (Custom/Mock)
    ↓
Phase 5-6: Validation & Testing
    ↓
Phase 7+: Real Simulator Integration (esmini, CarMaker)
```

---

### Phase 1: Middleware Data Structures (1-2 days)

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

### Phase 2: Mock Simulator Adapter (1 day)

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

### Phase 3: UDP IPC Adapter (1-2 days)

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

### Phase 4: Middleware Engine Core (2 days)

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

### Phase 5: Custom Test Application (1 day)

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

### Phase 6: Integration Testing & Validation (1 day)

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

### Phase 7: Real Simulator Integration - esmini (2-3 days)

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

### Phase 8: Additional IPC Protocols (2-3 days)

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

### Phase 9: CarMaker Integration (2-3 days)

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

### Phase 10: Production Hardening (2-3 days)

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

---

## Implementation

See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for detailed phased implementation strategy.
