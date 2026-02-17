#pragma once

#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include <vector>
#include <memory>

namespace middleware::simulator {

/**
 * Abstract simulator interface
 * 
 * All simulators (mock, esmini, CarMaker, etc.) implement this interface
 * to provide a uniform API for the middleware layer.
 */
class ISimulator {
public:
    virtual ~ISimulator() = default;
    
    /**
     * Initialize the simulator
     * @return true if initialization successful
     */
    virtual bool initialize() = 0;
    
    /**
     * Step the simulation forward by dt seconds
     * @param dt Time step in seconds
     * @return true if step successful
     */
    virtual bool step(double dt) = 0;
    
    /**
     * Get all vehicles in the simulation
     * @return Vector of vehicle states
     */
    virtual std::vector<data::VehicleState> getVehicles() const = 0;
    
    /**
     * Get ego vehicle (vehicle ID 0 by default)
     * @return Ego vehicle state
     */
    virtual data::VehicleState getEgoVehicle() const = 0;
    
    /**
     * Get current scene state
     * @return Scene state (weather, road conditions, etc.)
     */
    virtual data::SceneState getScene() const = 0;
    
    /**
     * Apply control command to a vehicle
     * @param cmd Control command to apply
     * @return true if command applied successfully
     */
    virtual bool applyControl(const data::ControlCommand& cmd) = 0;
    
    /**
     * Reset simulation to initial state
     * @return true if reset successful
     */
    virtual bool reset() = 0;
    
    /**
     * Get current simulation time
     * @return Simulation time in seconds
     */
    virtual double getSimulationTime() const = 0;
    
    /**
     * Check if simulation is running
     * @return true if simulation is active
     */
    virtual bool isRunning() const = 0;
    
    /**
     * Terminate the simulation
     */
    virtual void terminate() = 0;
};

/**
 * Simulator factory
 * Creates simulator instances based on configuration
 */
class SimulatorFactory {
public:
    enum class Type {
        MOCK,           // Simple mock simulator
        ESMINI,         // esmini (OpenSCENARIO)
        CARMAKER,       // IPG CarMaker
        CARLA,          // CARLA simulator
        CUSTOM          // User-defined
    };
    
    /**
     * Create a simulator instance
     * @param type Simulator type
     * @return Unique pointer to simulator instance
     */
    static std::unique_ptr<ISimulator> create(Type type);
    
    /**
     * Create a simulator from configuration string
     * @param typeStr String representation of type ("mock", "esmini", etc.)
     * @return Unique pointer to simulator instance
     */
    static std::unique_ptr<ISimulator> create(const std::string& typeStr);
};

} // namespace middleware::simulator
