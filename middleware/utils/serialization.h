#pragma once

#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include "middleware/data/sensor_data.h"
#include <cstddef>
#include <cstring>
#include <vector>
#include <stdexcept>

namespace middleware::utils {

// ============================================================================
// Binary Serialization Helpers
// ============================================================================

// Serialize single VehicleState to binary buffer
inline size_t serialize(const middleware::data::VehicleState& state, 
                       uint8_t* buffer, 
                       size_t bufferSize) {
    if (bufferSize < sizeof(state)) {
        throw std::runtime_error("Buffer too small for VehicleState serialization");
    }
    std::memcpy(buffer, &state, sizeof(state));
    return sizeof(state);
}

// Deserialize single VehicleState from binary buffer
inline bool deserialize(const uint8_t* buffer, 
                       size_t bufferSize, 
                       middleware::data::VehicleState& state) {
    if (bufferSize < sizeof(state)) {
        return false;
    }
    std::memcpy(&state, buffer, sizeof(state));
    return true;
}

// Serialize vector of VehicleStates to binary buffer
inline size_t serializeVehicles(const std::vector<middleware::data::VehicleState>& vehicles,
                               uint8_t* buffer,
                               size_t bufferSize) {
    // Format: [count:uint32_t][vehicle1][vehicle2]...[vehicleN]
    size_t requiredSize = sizeof(uint32_t) + (vehicles.size() * sizeof(middleware::data::VehicleState));
    
    if (bufferSize < requiredSize) {
        throw std::runtime_error("Buffer too small for vehicle list serialization");
    }
    
    uint32_t count = static_cast<uint32_t>(vehicles.size());
    std::memcpy(buffer, &count, sizeof(count));
    
    size_t offset = sizeof(count);
    for (const auto& vehicle : vehicles) {
        std::memcpy(buffer + offset, &vehicle, sizeof(vehicle));
        offset += sizeof(vehicle);
    }
    
    return offset;
}

// Deserialize vector of VehicleStates from binary buffer
inline bool deserializeVehicles(const uint8_t* buffer,
                               size_t bufferSize,
                               std::vector<middleware::data::VehicleState>& vehicles) {
    if (bufferSize < sizeof(uint32_t)) {
        return false;
    }
    
    uint32_t count;
    std::memcpy(&count, buffer, sizeof(count));
    
    size_t requiredSize = sizeof(uint32_t) + (count * sizeof(middleware::data::VehicleState));
    if (bufferSize < requiredSize) {
        return false;
    }
    
    vehicles.clear();
    vehicles.reserve(count);
    
    size_t offset = sizeof(count);
    for (uint32_t i = 0; i < count; ++i) {
        middleware::data::VehicleState vehicle;
        std::memcpy(&vehicle, buffer + offset, sizeof(vehicle));
        vehicles.push_back(vehicle);
        offset += sizeof(vehicle);
    }
    
    return true;
}

// Serialize SceneState to binary buffer
inline size_t serialize(const middleware::data::SceneState& state,
                       uint8_t* buffer,
                       size_t bufferSize) {
    if (bufferSize < sizeof(state)) {
        throw std::runtime_error("Buffer too small for SceneState serialization");
    }
    std::memcpy(buffer, &state, sizeof(state));
    return sizeof(state);
}

// Deserialize SceneState from binary buffer
inline bool deserialize(const uint8_t* buffer,
                       size_t bufferSize,
                       middleware::data::SceneState& state) {
    if (bufferSize < sizeof(state)) {
        return false;
    }
    std::memcpy(&state, buffer, sizeof(state));
    return true;
}

// Serialize ControlCommand to binary buffer
inline size_t serialize(const middleware::data::ControlCommand& cmd,
                       uint8_t* buffer,
                       size_t bufferSize) {
    if (bufferSize < sizeof(cmd)) {
        throw std::runtime_error("Buffer too small for ControlCommand serialization");
    }
    std::memcpy(buffer, &cmd, sizeof(cmd));
    return sizeof(cmd);
}

// Deserialize ControlCommand from binary buffer
inline bool deserialize(const uint8_t* buffer,
                       size_t bufferSize,
                       middleware::data::ControlCommand& cmd) {
    if (bufferSize < sizeof(cmd)) {
        return false;
    }
    std::memcpy(&cmd, buffer, sizeof(cmd));
    return true;
}

// Serialize SensorData to binary buffer
inline size_t serialize(const middleware::data::SensorData& data,
                       uint8_t* buffer,
                       size_t bufferSize) {
    if (bufferSize < sizeof(data)) {
        throw std::runtime_error("Buffer too small for SensorData serialization");
    }
    std::memcpy(buffer, &data, sizeof(data));
    return sizeof(data);
}

// Deserialize SensorData from binary buffer
inline bool deserialize(const uint8_t* buffer,
                       size_t bufferSize,
                       middleware::data::SensorData& data) {
    if (bufferSize < sizeof(data)) {
        return false;
    }
    std::memcpy(&data, buffer, sizeof(data));
    return true;
}

// ============================================================================
// Helper Functions for Network Endianness (optional - for cross-platform)
// ============================================================================

// Note: For simplicity, Phase 1 assumes little-endian systems (x86/x64)
// In production, add proper endianness conversion for network protocols

} // namespace middleware::utils
