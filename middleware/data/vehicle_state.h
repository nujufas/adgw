#pragma once

#include <cstdint>
#include <cstring>

namespace middleware::data {

// Canonical vehicle state - simulator-independent
// Size: 192 bytes (fixed, optimized for network transmission)
struct VehicleState {
    // ========== Identity (16 bytes) ==========
    uint64_t id;                    // Unique vehicle identifier
    double timestamp;               // Simulation time [s]
    
    // ========== Pose (48 bytes) ==========
    double x;                       // Global X position [m]
    double y;                       // Global Y position [m]
    double z;                       // Global Z position [m]
    double heading;                 // Yaw angle [rad], 0 = East, counter-clockwise
    double pitch;                   // Pitch angle [rad]
    double roll;                    // Roll angle [rad]
    
    // ========== Velocity (48 bytes) ==========
    double vx;                      // Velocity in X direction [m/s]
    double vy;                      // Velocity in Y direction [m/s]
    double vz;                      // Velocity in Z direction [m/s]
    double speed;                   // Total speed [m/s]
    double acceleration;            // Longitudinal acceleration [m/s²]
    double yaw_rate;                // Angular velocity around Z axis [rad/s]
    
    // ========== Dimensions (32 bytes) ==========
    double length;                  // Vehicle length [m]
    double width;                   // Vehicle width [m]
    double height;                  // Vehicle height [m]
    double wheelbase;               // Distance between front and rear axles [m]
    
    // ========== Vehicle State (32 bytes) ==========
    double steering_angle;          // Steering wheel angle [rad]
    double throttle;                // Throttle pedal position [0-1]
    double brake;                   // Brake pedal position [0-1]
    double gear;                    // Current gear (0=N, 1-6=forward, -1=R) - double for alignment
    
    // ========== Flags and Status (16 bytes) ==========
    uint32_t vehicle_type;          // 0=car, 1=truck, 2=motorcycle, 3=pedestrian
    uint32_t control_mode;          // 0=manual, 1=autonomous, 2=assisted
    uint32_t lights;                // Bit flags: 0x01=headlights, 0x02=brake, 0x04=turn_left, 0x08=turn_right
    uint32_t reserved[1];           // Future use (pad to 192 bytes)
    
    // Default constructor - zero initialize
    VehicleState() {
        std::memset(this, 0, sizeof(VehicleState));
    }
    
    // Equality operator for testing
    bool operator==(const VehicleState& other) const {
        return std::memcmp(this, &other, sizeof(VehicleState)) == 0;
    }
    
    bool operator!=(const VehicleState& other) const {
        return !(*this == other);
    }
};

// Compile-time size verification
static_assert(sizeof(VehicleState) == 192, 
              "VehicleState must be exactly 192 bytes for network protocol compatibility");

// Ensure proper alignment
static_assert(alignof(VehicleState) <= 8, 
              "VehicleState alignment must be compatible with network transmission");

} // namespace middleware::data
