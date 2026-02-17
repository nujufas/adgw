#pragma once

#include "middleware/simulator/simulator_interface.h"
#include "esminiLib.hpp"
#include <string>
#include <vector>
#include <stdexcept>

namespace middleware::simulator {

/**
 * esmini Simulator Adapter
 * 
 * Wraps the esmini library to implement the ISimulator interface.
 * Translates between esmini's SE_ScenarioObjectState and middleware's VehicleState.
 */
class EsminiAdapter : public ISimulator {
public:
    /**
     * Constructor
     * @param scenarioFile Path to OpenSCENARIO (.xosc) file
     * @param headless Run without graphics (default: true)
     * @param disableControllers Disable built-in esmini controllers (default: true)
     */
    explicit EsminiAdapter(
        const std::string& scenarioFile,
        bool headless = true,
        bool disableControllers = true
    );
    
    ~EsminiAdapter() override;
    
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
     * Translate esmini object state to middleware vehicle state
     */
    data::VehicleState translateToVehicleState(const SE_ScenarioObjectState& esminiState) const;
    
    /**
     * Apply control command using esmini API
     */
    bool applySpeedControl(int objectId, double targetSpeed);
    bool applyPositionControl(int objectId, const data::ControlCommand& cmd);
    bool applyAccelerationControl(int objectId, double acceleration);
    bool applySteeringControl(int objectId, double steeringAngle);
    
    std::string scenarioFile_;
    bool headless_;
    bool disableControllers_;
    bool initialized_;
    bool running_;
    double simTime_;
    int egoId_;  // esmini object ID for ego vehicle (usually 0)
};

} // namespace middleware::simulator
