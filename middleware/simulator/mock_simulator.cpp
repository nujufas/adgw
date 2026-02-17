#include "middleware/simulator/mock_simulator.h"
#include <cstring>
#include <algorithm>
#include <string>

namespace middleware::simulator {

MockSimulator::MockSimulator() 
    : config_{}
    , simTime_(0.0)
    , running_(false)
    , initialized_(false) {
}

MockSimulator::MockSimulator(const Config& config)
    : config_(config)
    , simTime_(0.0)
    , running_(false)
    , initialized_(false) {
}

bool MockSimulator::initialize() {
    if (initialized_) {
        return true;
    }
    
    // Create vehicles
    vehicles_.clear();
    vehicles_.reserve(config_.numVehicles);
    
    // Ego vehicle at origin
    vehicles_.push_back(createVehicle(0, 0.0, 0.0, config_.initialEgoSpeed));
    
    // Additional vehicles (lead vehicles ahead on same lane)
    for (size_t i = 1; i < config_.numVehicles; ++i) {
        double x = config_.leadDistance * i;
        double speed = config_.initialLeadSpeed;
        vehicles_.push_back(createVehicle(i, x, 0.0, speed));
    }
    
    // Initialize scene state
    std::memset(&scene_, 0, sizeof(scene_));
    scene_.timestamp = 0.0;
    scene_.temperature = 20.0;        // 20°C
    scene_.visibility = 1000.0;       // 1km visibility
    scene_.friction = 0.8;            // Good road conditions
    scene_.speed_limit = 33.33;       // ~120 km/h
    scene_.num_vehicles = static_cast<uint32_t>(vehicles_.size());
    scene_.traffic_density = 1;       // Light traffic
    
    simTime_ = 0.0;
    running_ = true;
    initialized_ = true;
    
    return true;
}

bool MockSimulator::step(double dt) {
    if (!running_ || !initialized_) {
        return false;
    }
    
    // Update each vehicle's physics
    for (auto& vehicle : vehicles_) {
        updateVehiclePhysics(vehicle, dt);
    }
    
    // Update simulation time
    simTime_ += dt;
    
    // Update scene state
    scene_.timestamp = simTime_;
    scene_.num_vehicles = static_cast<uint32_t>(vehicles_.size());
    
    return true;
}

std::vector<data::VehicleState> MockSimulator::getVehicles() const {
    return vehicles_;
}

data::VehicleState MockSimulator::getEgoVehicle() const {
    if (vehicles_.empty()) {
        return data::VehicleState{};
    }
    return vehicles_[0];
}

data::SceneState MockSimulator::getScene() const {
    return scene_;
}

bool MockSimulator::applyControl(const data::ControlCommand& cmd) {
    if (!running_ || !initialized_) {
        return false;
    }
    
    // Find the vehicle to control
    auto it = std::find_if(vehicles_.begin(), vehicles_.end(),
        [&cmd](const data::VehicleState& v) { return v.id == cmd.vehicleId; });
    
    if (it == vehicles_.end()) {
        return false;  // Vehicle not found
    }
    
    auto& vehicle = *it;
    
    // Apply control based on mode
    using Mode = data::ControlCommand::Mode;
    
    switch (cmd.mode) {
        case Mode::SPEED:
            // Speed control - adjust velocity to match target
            applySpeedControl(vehicle, cmd.speed.targetSpeed, 
                            cmd.speed.maxAcceleration > 0 ? cmd.speed.maxAcceleration : 3.0,
                            1.0 / config_.updateRate);
            break;
            
        case Mode::ACCELERATION:
            // Acceleration control
            applyAccelerationControl(vehicle, cmd.acceleration.targetAcceleration,
                                   1.0 / config_.updateRate);
            break;
            
        case Mode::POSITION:
            // Position control (teleportation)
            applyPositionControl(vehicle, cmd);
            break;
            
        case Mode::THROTTLE_BRAKE:
            // Direct throttle/brake control
            vehicle.throttle = cmd.throttleBrake.throttle;
            vehicle.brake = cmd.throttleBrake.brake;
            // Simple model: throttle accelerates, brake decelerates
            if (vehicle.throttle > 0.1) {
                vehicle.acceleration = vehicle.throttle * 3.0;  // Max 3 m/s²
            } else if (vehicle.brake > 0.1) {
                vehicle.acceleration = -vehicle.brake * 5.0;   // Max -5 m/s² braking
            }
            break;
            
        case Mode::FULL_STATE:
            // Full state override
            vehicle.vx = cmd.fullState.vx;
            vehicle.vy = cmd.fullState.vy;
            vehicle.speed = std::sqrt(vehicle.vx * vehicle.vx + vehicle.vy * vehicle.vy);
            vehicle.yaw_rate = cmd.fullState.yawRate;
            vehicle.steering_angle = cmd.fullState.steeringAngle;
            vehicle.throttle = cmd.fullState.throttle;
            vehicle.brake = cmd.fullState.brake;
            vehicle.gear = cmd.fullState.gear;
            break;
            
        default:
            return false;  // Unsupported mode
    }
    
    return true;
}

bool MockSimulator::reset() {
    vehicles_.clear();
    simTime_ = 0.0;
    running_ = false;
    initialized_ = false;
    return initialize();
}

double MockSimulator::getSimulationTime() const {
    return simTime_;
}

bool MockSimulator::isRunning() const {
    return running_;
}

void MockSimulator::terminate() {
    running_ = false;
}

// Private helper methods

data::VehicleState MockSimulator::createVehicle(uint64_t id, double x, double y, double speed) {
    data::VehicleState vehicle;
    std::memset(&vehicle, 0, sizeof(vehicle));
    
    // Identity
    vehicle.id = id;
    vehicle.timestamp = simTime_;
    
    // Pose
    vehicle.x = x;
    vehicle.y = y;
    vehicle.z = 0.0;
    vehicle.heading = 0.0;  // East direction
    vehicle.pitch = 0.0;
    vehicle.roll = 0.0;
    
    // Velocity
    vehicle.vx = speed;
    vehicle.vy = 0.0;
    vehicle.vz = 0.0;
    vehicle.speed = speed;
    vehicle.acceleration = 0.0;
    vehicle.yaw_rate = 0.0;
    
    // Dimensions (typical passenger car)
    vehicle.length = 4.5;
    vehicle.width = 1.8;
    vehicle.height = 1.5;
    vehicle.wheelbase = 2.7;
    
    // Vehicle state
    vehicle.steering_angle = 0.0;
    vehicle.throttle = 0.0;
    vehicle.brake = 0.0;
    vehicle.gear = 1.0;  // First gear
    
    // Flags
    vehicle.vehicle_type = 0;  // Car
    vehicle.control_mode = id == 0 ? 1 : 0;  // Ego is autonomous, others manual
    vehicle.lights = 0x01;  // Headlights on
    
    return vehicle;
}

void MockSimulator::updateVehiclePhysics(data::VehicleState& vehicle, double dt) {
    // Simple kinematic model: integrate velocity to get position
    
    // Update velocity from acceleration
    vehicle.speed += vehicle.acceleration * dt;
    vehicle.speed = std::max(0.0, vehicle.speed);  // No negative speed
    
    // Update velocity components (assuming straight motion)
    vehicle.vx = vehicle.speed * std::cos(vehicle.heading);
    vehicle.vy = vehicle.speed * std::sin(vehicle.heading);
    
    // Update position
    vehicle.x += vehicle.vx * dt;
    vehicle.y += vehicle.vy * dt;
    
    // Update heading from yaw rate
    vehicle.heading += vehicle.yaw_rate * dt;
    
    // Normalize heading to [-π, π]
    while (vehicle.heading > M_PI) vehicle.heading -= 2.0 * M_PI;
    while (vehicle.heading < -M_PI) vehicle.heading += 2.0 * M_PI;
    
    // Update timestamp
    vehicle.timestamp = simTime_;
}

void MockSimulator::applySpeedControl(data::VehicleState& vehicle, double targetSpeed, 
                                     double maxAccel, double dt) {
    double speedError = targetSpeed - vehicle.speed;
    
    // Simple proportional control
    double desiredAccel = speedError * 2.0;  // Proportional gain
    
    // Clamp acceleration
    if (desiredAccel > maxAccel) {
        desiredAccel = maxAccel;
    } else if (desiredAccel < -maxAccel) {
        desiredAccel = -maxAccel;
    }
    
    vehicle.acceleration = desiredAccel;
}

void MockSimulator::applyPositionControl(data::VehicleState& vehicle, 
                                        const data::ControlCommand& cmd) {
    // Teleport vehicle to target position
    vehicle.x = cmd.position.x;
    vehicle.y = cmd.position.y;
    vehicle.z = cmd.position.z;
    vehicle.heading = cmd.position.heading;
    vehicle.pitch = cmd.position.pitch;
    vehicle.roll = cmd.position.roll;
}

void MockSimulator::applyAccelerationControl(data::VehicleState& vehicle, 
                                            double targetAccel, double dt) {
    vehicle.acceleration = targetAccel;
}

// Simulator factory implementation

std::unique_ptr<ISimulator> SimulatorFactory::create(Type type) {
    switch (type) {
        case Type::MOCK:
            return std::make_unique<MockSimulator>();
        
        case Type::ESMINI:
        case Type::CARMAKER:
        case Type::CARLA:
        case Type::CUSTOM:
            // Not implemented yet
            return nullptr;
        
        default:
            return nullptr;
    }
}

std::unique_ptr<ISimulator> SimulatorFactory::create(const std::string& typeStr) {
    if (typeStr == "mock") {
        return create(Type::MOCK);
    } else if (typeStr == "esmini") {
        return create(Type::ESMINI);
    } else if (typeStr == "carmaker") {
        return create(Type::CARMAKER);
    } else if (typeStr == "carla") {
        return create(Type::CARLA);
    } else {
        return nullptr;
    }
}

} // namespace middleware::simulator
