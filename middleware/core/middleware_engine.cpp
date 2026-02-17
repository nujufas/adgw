#include "middleware/core/middleware_engine.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

namespace middleware::core {

MiddlewareEngine::MiddlewareEngine()
    : config_{}
    , running_(false)
    , initialized_(false)
    , startTime_(0.0)
    , lastStepTime_(0.0)
    , stats_{}
    , totalStepTime_(0.0)
    , totalLoopTime_(0.0)
{
}

MiddlewareEngine::MiddlewareEngine(const EngineConfig& config)
    : config_(config)
    , running_(false)
    , initialized_(false)
    , startTime_(0.0)
    , lastStepTime_(0.0)
    , stats_{}
    , totalStepTime_(0.0)
    , totalLoopTime_(0.0)
{
}

MiddlewareEngine::~MiddlewareEngine() {
    stop();
}

bool MiddlewareEngine::initialize() {
    if (initialized_) {
        return true;
    }
    
    // Validate we have a simulator
    if (!simulator_) {
        std::cerr << "Error: No simulator configured" << std::endl;
        return false;
    }
    
    // Initialize simulator
    if (!simulator_->initialize()) {
        std::cerr << "Error: Failed to initialize simulator" << std::endl;
        return false;
    }
    
    // Initialize all IPC adapters
    for (auto& adapter : ipcAdapters_) {
        if (!adapter->initialize()) {
            std::cerr << "Error: Failed to initialize IPC adapter" << std::endl;
            return false;
        }
        
        // Register command callback if control is enabled
        if (config_.enableControlCommands) {
            // Capture 'this' to handle commands
            adapter->registerCommandCallback(
                [this](const data::ControlCommand& cmd) {
                    handleControlCommand(cmd);
                }
            );
            
            // Start async listener
            if (!adapter->startAsyncListener()) {
                std::cerr << "Warning: Failed to start async listener" << std::endl;
            }
        }
    }
    
    // Record start time
    startTime_ = getCurrentTime();
    lastStepTime_ = startTime_;
    
    initialized_ = true;
    
    if (config_.enableLogging) {
        std::cout << "Middleware Engine initialized" << std::endl;
        std::cout << "  Simulator: " << config_.simulatorType << std::endl;
        std::cout << "  Timestep: " << config_.timestep << " s (" 
                  << (1.0 / config_.timestep) << " Hz)" << std::endl;
        std::cout << "  IPC adapters: " << ipcAdapters_.size() << std::endl;
    }
    
    return true;
}

void MiddlewareEngine::run() {
    if (!initialized_) {
        std::cerr << "Error: Engine not initialized" << std::endl;
        return;
    }
    
    running_.store(true);
    
    if (config_.enableLogging) {
        std::cout << "Middleware Engine running..." << std::endl;
    }
    
    while (running_.load()) {
        mainLoopIteration();
    }
    
    if (config_.enableLogging) {
        std::cout << "Middleware Engine stopped" << std::endl;
    }
}

void MiddlewareEngine::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    
    // Stop all IPC async listeners
    for (auto& adapter : ipcAdapters_) {
        adapter->stopAsyncListener();
    }
    
    // Terminate simulator
    if (simulator_) {
        simulator_->terminate();
    }
    
    // Terminate IPC adapters
    for (auto& adapter : ipcAdapters_) {
        adapter->terminate();
    }
    
    initialized_ = false;
}

bool MiddlewareEngine::isRunning() const {
    return running_.load();
}

void MiddlewareEngine::mainLoopIteration() {
    double loopStartTime = getCurrentTime();
    
    // 1. Step simulator
    double stepStartTime = getCurrentTime();
    bool stepped = simulator_->step(config_.timestep);
    double stepEndTime = getCurrentTime();
    double stepTime = stepEndTime - stepStartTime;
    
    if (!stepped) {
        std::cerr << "Warning: Simulator step failed" << std::endl;
    }
    
    // Check if step time exceeded limit
    if (config_.enableValidation && stepTime > config_.maxStepTime) {
        std::cerr << "Warning: Step time " << stepTime 
                  << "s exceeded limit " << config_.maxStepTime << "s" << std::endl;
    }
    
    // 2. Publish perception data
    publishPerceptionData();
    
    // 3. Control commands are handled asynchronously via callbacks
    //    (no need to poll here)
    
    // 4. Update statistics
    double loopEndTime = getCurrentTime();
    double loopTime = loopEndTime - loopStartTime;
    updateStatistics(loopTime, stepTime);
    
    // 5. Rate limiting - sleep until next iteration
    double targetTime = loopStartTime + config_.timestep;
    sleepUntil(targetTime);
    
    lastStepTime_ = loopEndTime;
}

void MiddlewareEngine::publishPerceptionData() {
    // Get vehicle states from simulator
    auto vehicles = simulator_->getVehicles();
    
    // Get scene state from simulator
    auto scene = simulator_->getScene();
    
    // Publish to all IPC adapters
    for (auto& adapter : ipcAdapters_) {
        adapter->publishVehicles(vehicles);
        adapter->publishScene(scene);
    }
}

void MiddlewareEngine::handleControlCommand(const data::ControlCommand& cmd) {
    stats_.commandsReceived++;
    
    // Validate command if enabled
    if (config_.enableValidation) {
        if (!validateCommand(cmd)) {
            stats_.commandsRejected++;
            if (config_.enableLogging) {
                std::cerr << "Rejected invalid command for vehicle " 
                          << cmd.vehicleId << std::endl;
            }
            return;
        }
    }
    
    // Apply command to simulator
    bool applied = simulator_->applyControl(cmd);
    
    if (applied) {
        stats_.commandsApplied++;
    } else {
        stats_.commandsRejected++;
        if (config_.enableLogging) {
            std::cerr << "Failed to apply command for vehicle " 
                      << cmd.vehicleId << std::endl;
        }
    }
}

bool MiddlewareEngine::validateCommand(const data::ControlCommand& cmd) const {
    // Basic validation checks
    
    // Check for valid vehicle ID
    auto vehicles = simulator_->getVehicles();
    bool validVehicle = false;
    for (const auto& v : vehicles) {
        if (v.id == cmd.vehicleId) {
            validVehicle = true;
            break;
        }
    }
    
    if (!validVehicle) {
        return false;
    }
    
    // Check timestamp is not too old
    double simTime = simulator_->getSimulationTime();
    if (cmd.timestamp > 0.0 && (simTime - cmd.timestamp) > 1.0) {
        // Command is more than 1 second old
        return false;
    }
    
    // Validate mode-specific parameters
    switch (cmd.mode) {
        case data::ControlCommand::Mode::SPEED:
            if (cmd.speed.targetSpeed < 0.0 || cmd.speed.targetSpeed > 100.0) {
                return false;
            }
            break;
            
        case data::ControlCommand::Mode::ACCELERATION:
            if (std::abs(cmd.acceleration.targetAcceleration) > 10.0) {
                return false;
            }
            break;
            
        case data::ControlCommand::Mode::THROTTLE_BRAKE:
            if (cmd.throttleBrake.throttle < 0.0 || cmd.throttleBrake.throttle > 1.0 ||
                cmd.throttleBrake.brake < 0.0 || cmd.throttleBrake.brake > 1.0) {
                return false;
            }
            break;
            
        default:
            break;
    }
    
    return true;
}

void MiddlewareEngine::updateStatistics(double loopTime, double stepTime) {
    stats_.totalCycles++;
    
    totalStepTime_ += stepTime;
    totalLoopTime_ += loopTime;
    
    // Update averages
    stats_.averageStepTime = totalStepTime_ / stats_.totalCycles;
    stats_.averageLoopTime = totalLoopTime_ / stats_.totalCycles;
    
    // Update max
    if (stepTime > stats_.maxStepTime) {
        stats_.maxStepTime = stepTime;
    }
    
    // Update simulation time
    stats_.currentSimTime = simulator_->getSimulationTime();
    
    // Update real-time factor
    double currentTime = getCurrentTime();
    stats_.realTimeElapsed = currentTime - startTime_;
    if (stats_.realTimeElapsed > 0.0) {
        stats_.realTimeFactor = stats_.currentSimTime / stats_.realTimeElapsed;
    }
}

double MiddlewareEngine::getCurrentTime() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

void MiddlewareEngine::sleepUntil(double targetTime) {
    double currentTime = getCurrentTime();
    double sleepTime = targetTime - currentTime;
    
    if (sleepTime > 0.0) {
        auto sleepDuration = std::chrono::duration<double>(sleepTime);
        std::this_thread::sleep_for(sleepDuration);
    }
}

EngineStatistics MiddlewareEngine::getStatistics() const {
    return stats_;
}

void MiddlewareEngine::setSimulator(std::unique_ptr<simulator::ISimulator> sim) {
    simulator_ = std::move(sim);
}

void MiddlewareEngine::addIPCAdapter(std::unique_ptr<ipc::IIPCAdapter> adapter) {
    ipcAdapters_.push_back(std::move(adapter));
}

} // namespace middleware::core
