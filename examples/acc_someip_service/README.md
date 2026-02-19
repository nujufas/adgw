# ACC SomeIP Service

This directory contains the FIDL-based SomeIP service for ACC (Adaptive Cruise Control) integration with the esmini middleware.

## Directory Structure

```
acc_someip_service/
├── fidl/                           # Franca IDL interface definitions
│   └── accServices.fidl           # Service interface definitions
├── fdepl/                          # Franca deployment specifications
│   └── accServices-someip.fdepl   # SOME/IP deployment configuration
├── config/                         # Runtime configuration files
│   ├── vsomeip.json              # vsomeip configuration
│   └── commonapi.ini             # CommonAPI configuration
├── src-gen/                        # Generated code (created by code generator)
├── src/                           # Application source code
│   └── accServer/                # Server implementation
├── CMakeLists.txt                 # CMake build configuration
└── README.md                      # This file
```

## Overview

This service provides two interfaces:

1. **accInputServiceInterface** (Service ID 0x1234):
   - Receives vehicle state and target information from esmini middleware
   - 10 attributes for vehicle speed, acceleration, target position, etc.
   
2. **accOutputServiceInterface** (Service ID 0x1235):
   - Provides ACC control outputs back to the middleware
   - 9 attributes for set speeds, gap control, torque commands, etc.

## Architecture

```
┌──────────────┐                    ┌──────────────────┐
│              │  VehicleState      │                  │
│   esmini    │──SomeIP Client─────>│  accServer       │
│  Middleware  │                    │  (SomeIP Server) │
│              │<──ControlCommand──│                  │
└──────────────┘                    └──────────────────┘
     |                                      |
     |                                      |
 someip_adapter.cpp                  accInputServiceStubImpl.cpp
```

## Code Generation

The project uses CommonAPI code generators to create SomeIP bindings from FIDL files.

### Prerequisites

- CommonAPI Core Generator (>= 3.2.0)
- CommonAPI SomeIP Generator (>= 3.2.0)
- CommonAPI Runtime (>= 3.2.0)
- CommonAPI SomeIP Runtime (>= 3.2.0)
- vsomeip3 (>= 3.1.0)

### Generate Code

```bash
# Core generator (creates proxy/stub base classes)
commonapi-core-generator-linux-x86_64 -sk -d src-gen fidl/accServices.fidl

# SomeIP generator (creates SomeIP-specific binding)
commonapi-someip-generator-linux-x86_64 -d src-gen fdepl/accServices-someip.fdepl
```

This will generate the following in `src-gen/`:
- Proxy classes (for clients)
- Stub classes (for servers)
- SomeIP adapters and deployment files

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Run

### Start the Server

```bash
# Export configuration paths
export COMMONAPI_CONFIG=../config/commonapi.ini
export VSOMEIP_CONFIGURATION=../config/vsomeip.json

# Run server
./accServer
```

## Integration with Middleware

The middleware uses the `SomeIPAdapter` class (in `middleware/ipc/someip_adapter.cpp`) to:
1. Connect as a SomeIP client to the accServer
2. Convert esmini `VehicleState` data to SomeIP attributes
3. Update the server's attributes via CommonAPI
4. Receive control commands from the ACC application

## FIDL to C++ Mapping

| FIDL Type | C++ Type | Description |
|-----------|----------|-------------|
| Double    | double   | 64-bit floating point |
| Float     | float    | 32-bit floating point |
| Boolean   | bool     | Boolean value |
| attribute | getter/setter + onChange event | Readable/writable property |

## Service IDs

| Service | Service ID | Instance ID | Port |
|---------|------------|-------------|------|
| accInputServiceInterface | 0x1234 | 0x0001 | 30501 |
| accOutputServiceInterface | 0x1235 | 0x0001 | 30502 |

## Configuration

### vsomeip.json
- Defines service IDs, instance IDs, and network ports
- Configures service discovery settings
- Sets up routing and logging

### commonapi.ini
- Specifies the CommonAPI binding (someip)
- Points to the CommonAPI-SomeIP library

## Data Flow

1. **esmini → middleware**: Simulator generates vehicle states
2. **middleware → SomeIP Client**: SomeIP adapter converts data
3. **SomeIP Client → accServer**: Updates service attributes
4. **accServer → ACC Logic**: Processes sensor data
5. **ACC Logic → accServer**: Generates control outputs
6. **accServer → middleware**: Control commands via SomeIP
7. **middleware → esmini**: Apply control to simulation
