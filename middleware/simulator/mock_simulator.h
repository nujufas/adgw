#pragma once

#include "middleware/simulator/simulator_interface.h"
#include <vector>
#include <cmath>

namespace middleware::simulator {

/**
 * Mock simulator for testing
 * 
 * Implements a simple kinematic vehicle model with:
 * - Position integration (x, y from velocity)
 * - Velocity control (responds to speed commands)
 * - Multiple vehicles support
 * - Deterministic behavior for testing
 */
class MockSimulator : public ISimulator {
public:
    struct Config {
        size_t numVehicles = 2;          // Number of vehicles (ego + others)
        double initialEgoSpeed = 15.0;   // Ego initial speed [m/s]
        double initialLeadSpeed = 20.0;  // Lead vehicle speed [m/s]
        double leadDistance = 50.0;      // Initial distance between vehicles [m]
        double updateRate = 100.0;       // Simulation update rate [Hz]
    };
    
    /**
     * Create mock simulator with default configuration
     */
    MockSimulator();
    
    /**
     * Create mock simulator with custom configuration
     * @param config Simulator configuration
     */
    explicit MockSimulator(const Config& config);
    
    ~MockSimulator() override = default;
    
    // ISimulator interface implementation
    bool initialize() override;
    bool step(double dt) override;
    std::vector<data::VehicleState> getVehicles() const override;
    data::VehicleState getEgoVehicle() const override;
    data::SceneState getScene() const override;
    bool applyControl(const data::ControlCommand& cmd) override;
    bool reset() override;
    double getSimulationTime() const override;
    bool isRunning() const override;
    void terminate() override;
    
private:
    /**
     * Create a vehicle with initial state
     */
    data::VehicleState createVehicle(uint64_t id, double x, double y, double speed);
    
    /**
     * Update vehicle physics (simple kinematic model)
     */
    void updateVehiclePhysics(data::VehicleState& vehicle, double dt);
    
    /**
     * Apply speed control to vehicle
     */
    void applySpeedControl(data::VehicleState& vehicle, double targetSpeed, double maxAccel, double dt);
    
    /**
     * Apply position control to vehicle (teleportation)
     */
    void applyPositionControl(data::VehicleState& vehicle, const data::ControlCommand& cmd);
    
    /**
     * Apply acceleration control to vehicle
     */
    void applyAccelerationControl(data::VehicleState& vehicle, double targetAccel, double dt);
    
    Config config_;
    std::vector<data::VehicleState> vehicles_;
    data::SceneState scene_;
    double simTime_;
    bool running_;
    bool initialized_;
};

} // namespace middleware::simulator
