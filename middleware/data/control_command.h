#pragma once

#include <cstdint>
#include <cstring>

namespace middleware::data {

// Control command - how applications command vehicle behavior
// Size: ≤512 bytes (supports all control modes)
struct ControlCommand {
    // ========== Header (24 bytes) ==========
    uint64_t vehicleId;             // Target vehicle ID
    double timestamp;               // Command timestamp [s]
    uint64_t sequenceNumber;        // Command sequence for ordering
    
    // ========== Control Mode (4 bytes) ==========
    enum class Mode : uint32_t {
        NONE = 0,                   // No control
        SPEED = 1,                  // Speed control (cruise control, ACC)
        ACCELERATION = 2,           // Acceleration control
        POSITION = 3,               // Position control (teleportation)
        TRAJECTORY = 4,             // Trajectory following
        STEERING = 5,               // Direct steering control
        THROTTLE_BRAKE = 6,         // Direct pedal control
        FULL_STATE = 7              // Complete vehicle state override
    };
    Mode mode;
    
    // ========== Speed Control (32 bytes) ==========
    struct {
        double targetSpeed;         // Target speed [m/s]
        double speedTolerance;      // Acceptable speed deviation [m/s]
        double maxAcceleration;     // Max acceleration [m/s²]
        double maxDeceleration;     // Max deceleration [m/s²]
    } speed;
    
    // ========== Acceleration Control (16 bytes) ==========
    struct {
        double targetAcceleration;  // Target longitudinal acceleration [m/s²]
        double duration;            // Duration to apply [s]
    } acceleration;
    
    // ========== Position Control (48 bytes) ==========
    struct {
        double x;                   // Target X position [m]
        double y;                   // Target Y position [m]
        double z;                   // Target Z position [m]
        double heading;             // Target heading [rad]
        double pitch;               // Target pitch [rad]
        double roll;                // Target roll [rad]
    } position;
    
    // ========== Trajectory Control (256 bytes) ==========
    struct {
        uint32_t numPoints;         // Number of waypoints (max 10)
        uint32_t padding;
        struct Waypoint {
            double x;               // Waypoint X [m]
            double y;               // Waypoint Y [m]
            double speed;           // Target speed at waypoint [m/s]
        } waypoints[10];            // Array of waypoints
    } trajectory;
    
    // ========== Steering Control (16 bytes) ==========
    struct {
        double steeringAngle;       // Target steering angle [rad]
        double steeringRate;        // Rate of steering change [rad/s]
    } steering;
    
    // ========== Throttle/Brake Control (16 bytes) ==========
    struct {
        double throttle;            // Throttle position [0-1]
        double brake;               // Brake position [0-1]
    } throttleBrake;
    
    // ========== Full State Control (64 bytes) ==========
    struct {
        double vx;                  // Target velocity X [m/s]
        double vy;                  // Target velocity Y [m/s]
        double yawRate;             // Target yaw rate [rad/s]
        double throttle;            // Throttle [0-1]
        double brake;               // Brake [0-1]
        double steeringAngle;       // Steering angle [rad]
        uint32_t gear;              // Gear selection
        uint32_t padding;
    } fullState;
    
    // ========== Validation and Safety (16 bytes) ==========
    uint32_t flags;                 // Bit flags: 0x01=urgent, 0x02=override_safety
    double maxExecutionTime;        // Max time to execute command [s]
    uint32_t reserved[4];           // Future use
    
    // Default constructor - zero initialize
    ControlCommand() {
        std::memset(this, 0, sizeof(ControlCommand));
        mode = Mode::NONE;
    }
    
    // Equality operator for testing
    bool operator==(const ControlCommand& other) const {
        return std::memcmp(this, &other, sizeof(ControlCommand)) == 0;
    }
    
    bool operator!=(const ControlCommand& other) const {
        return !(*this == other);
    }
};

// Compile-time size verification
static_assert(sizeof(ControlCommand) <= 512, 
              "ControlCommand must be at most 512 bytes for efficient transmission");

} // namespace middleware::data
