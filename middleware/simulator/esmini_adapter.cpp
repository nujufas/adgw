#include "middleware/simulator/esmini_adapter.h"
#include <cmath>
#include <iostream>

namespace middleware::simulator {

EsminiAdapter::EsminiAdapter(
    const std::string& scenarioFile,
    bool headless,
    bool disableControllers
) : scenarioFile_(scenarioFile),
    headless_(headless),
    disableControllers_(disableControllers),
    initialized_(false),
    running_(false),
    simTime_(0.0),
    egoId_(0)
{
}

EsminiAdapter::~EsminiAdapter() {
    terminate();
}

bool EsminiAdapter::initialize() {
    if (initialized_) {
        std::cerr << "EsminiAdapter already initialized\n";
        return true;
    }
    
    // SE_Init parameters:
    // - scenario file
    // - use_viewer (0 = headless, 1 = with graphics)
    // - threads (0 = no threading)
    // - record (0 = no recording)
    // - headstart_time (0 = no headstart)
    int useViewer = headless_ ? 0 : 1;
    
    if (SE_Init(scenarioFile_.c_str(), useViewer, 0, 0, 0) != 0) {
        std::cerr << "Failed to initialize esmini with scenario: " << scenarioFile_ << "\n";
        return false;
    }
    
    // Disable built-in controllers if requested (for external control)
    if (disableControllers_) {
        // Note: In esmini, we need to explicitly take control of objects
        // This is typically done per-object when applying control
        int numObjects = SE_GetNumberOfObjects();
        for (int i = 0; i < numObjects; i++) {
            // Setting external control mode
            // We'll do this when applyControl is called
        }
    }
    
    initialized_ = true;
    running_ = true;
    simTime_ = 0.0;
    
    std::cout << "EsminiAdapter initialized successfully\n";
    std::cout << "  Scenario: " << scenarioFile_ << "\n";
    std::cout << "  Headless: " << (headless_ ? "yes" : "no") << "\n";
    std::cout << "  Number of objects: " << SE_GetNumberOfObjects() << "\n";
    
    return true;
}

bool EsminiAdapter::step(double dt) {
    if (!initialized_ || !running_) {
        return false;
    }
    
    // Step esmini simulation
    // SE_Step takes no parameters - it uses the dt defined in the scenario
    // We'll call SE_StepDT to use our custom timestep
    if (SE_StepDT(static_cast<float>(dt)) != 0) {
        running_ = false;
        return false;
    }
    
    // Update simulation time
    simTime_ = SE_GetSimulationTime();
    
    return true;
}

std::vector<data::VehicleState> EsminiAdapter::getVehicles() const {
    if (!initialized_) {
        return {};
    }
    
    std::vector<data::VehicleState> vehicles;
    int numObjects = SE_GetNumberOfObjects();
    
    for (int i = 0; i < numObjects; i++) {
        SE_ScenarioObjectState esminiState;
        if (SE_GetObjectState(i, &esminiState) == 0) {
            vehicles.push_back(translateToVehicleState(esminiState));
        }
    }
    
    return vehicles;
}

data::VehicleState EsminiAdapter::getEgoVehicle() const {
    if (!initialized_) {
        return data::VehicleState();
    }
    
    SE_ScenarioObjectState esminiState;
    if (SE_GetObjectState(egoId_, &esminiState) == 0) {
        return translateToVehicleState(esminiState);
    }
    
    return data::VehicleState();
}

data::SceneState EsminiAdapter::getScene() const {
    // Create a basic scene state
    // esmini doesn't directly provide weather/lighting info,
    // but we can populate with defaults or extract from scenario
    data::SceneState scene;
    scene.timestamp = simTime_;
    scene.num_vehicles = SE_GetNumberOfObjects();
    scene.weather_type = 0;  // Default: clear
    scene.ambient_light = 10000.0;  // Default: day (10k lux)
    scene.friction = 0.8;  // Default: dry road
    
    return scene;
}

bool EsminiAdapter::applyControl(const data::ControlCommand& cmd) {
    if (!initialized_) {
        return false;
    }
    
    int objectId = static_cast<int>(cmd.vehicleId);
    
    // Apply control based on command mode
    switch (cmd.mode) {
        case data::ControlCommand::Mode::SPEED:
            return applySpeedControl(objectId, cmd.speed.targetSpeed);
            
        case data::ControlCommand::Mode::ACCELERATION:
            return applyAccelerationControl(objectId, cmd.acceleration.targetAcceleration);
            
        case data::ControlCommand::Mode::STEERING:
            return applySteeringControl(objectId, cmd.steering.steeringAngle);
            
        case data::ControlCommand::Mode::POSITION:
            return applyPositionControl(objectId, cmd);
            
        default:
            std::cerr << "Unsupported control mode: " << static_cast<int>(cmd.mode) << "\n";
            return false;
    }
}

bool EsminiAdapter::reset() {
    if (!initialized_) {
        return false;
    }
    
    // Close current simulation
    SE_Close();
    
    // Re-initialize
    initialized_ = false;
    running_ = false;
    simTime_ = 0.0;
    
    return initialize();
}

double EsminiAdapter::getSimulationTime() const {
    return simTime_;
}

bool EsminiAdapter::isRunning() const {
    return running_;
}

void EsminiAdapter::terminate() {
    if (initialized_) {
        SE_Close();
        initialized_ = false;
        running_ = false;
    }
}

// ============================================================================
// Private helper methods
// ============================================================================

data::VehicleState EsminiAdapter::translateToVehicleState(
    const SE_ScenarioObjectState& esminiState
) const {
    data::VehicleState v;
    
    // Identity
    v.id = static_cast<uint64_t>(esminiState.id);
    v.timestamp = esminiState.timestamp;
    
    // Pose
    v.x = esminiState.x;
    v.y = esminiState.y;
    v.z = esminiState.z;
    v.heading = esminiState.h;
    v.pitch = esminiState.p;
    v.roll = esminiState.r;
    
    // Velocity
    v.speed = esminiState.speed;
    v.vx = esminiState.speed * std::cos(esminiState.h);
    v.vy = esminiState.speed * std::sin(esminiState.h);
    v.vz = 0.0;  // esmini doesn't provide vz directly
    v.acceleration = 0.0;  // Not directly available, would need to compute from speed delta
    v.yaw_rate = 0.0;  // Not directly available
    
    // Dimensions
    v.length = esminiState.length;
    v.width = esminiState.width;
    v.height = esminiState.height;
    v.wheelbase = esminiState.length * 0.6;  // Estimate: ~60% of vehicle length
    
    // Vehicle state
    v.steering_angle = esminiState.wheel_angle;
    v.throttle = 0.0;  // Not available from esmini
    v.brake = 0.0;  // Not available from esmini
    v.gear = 1.0;  // Assume forward gear
    
    // Flags
    v.vehicle_type = static_cast<uint32_t>(esminiState.objectType);
    v.control_mode = (esminiState.ctrl_type == 1) ? 1 : 0;  // 1=External, 0=Internal
    v.lights = 0;
    
    return v;
}

bool EsminiAdapter::applySpeedControl(int objectId, double targetSpeed) {
    // Use SE_ReportObjectSpeed to set speed directly
    if (SE_ReportObjectSpeed(objectId, static_cast<float>(targetSpeed)) != 0) {
        std::cerr << "Failed to apply speed control to object " << objectId << "\n";
        return false;
    }
    return true;
}

bool EsminiAdapter::applyPositionControl(int objectId, const data::ControlCommand& cmd) {
    // Use SE_ReportObjectPos to set position directly
    // SE_ReportObjectPos signature: (object_id, timestamp, x, y, z, h, p, r)
    if (SE_ReportObjectPos(
        objectId,
        static_cast<float>(simTime_),  // timestamp parameter
        static_cast<float>(cmd.position.x),
        static_cast<float>(cmd.position.y),
        static_cast<float>(cmd.position.z),
        static_cast<float>(cmd.position.heading),
        static_cast<float>(cmd.position.pitch),
        static_cast<float>(cmd.position.roll)
    ) != 0) {
        std::cerr << "Failed to apply position control to object " << objectId << "\n";
        return false;
    }
    return true;
}

bool EsminiAdapter::applyAccelerationControl(int objectId, double acceleration) {
    // esmini doesn't have direct acceleration control
    // We need to compute target speed based on current speed + acceleration * dt
    SE_ScenarioObjectState state;
    if (SE_GetObjectState(objectId, &state) != 0) {
        return false;
    }
    
    // Use a default dt of 0.01s for acceleration integration
    double dt = 0.01;
    double newSpeed = state.speed + acceleration * dt;
    newSpeed = std::max(0.0, newSpeed);  // Don't allow negative speed
    
    return applySpeedControl(objectId, newSpeed);
}

bool EsminiAdapter::applySteeringControl(int /* objectId */, double /* steeringAngle */) {
    // esmini doesn't have direct steering control via its API
    // Steering is typically controlled through vehicle models or trajectories
    // For now, we'll just log this and return false
    std::cerr << "Direct steering control not supported by esmini API\n";
    return false;
}

} // namespace middleware::simulator
