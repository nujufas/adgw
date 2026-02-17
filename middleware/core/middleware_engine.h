#pragma once

#include "middleware/simulator/simulator_interface.h"
#include "middleware/ipc/ipc_interface.h"
#include "middleware/data/control_command.h"
#include <memory>
#include <vector>
#include <atomic>
#include <string>

namespace middleware::core {

// Configuration for middleware engine
struct EngineConfig {
    // Simulator settings
    std::string simulatorType = "mock";
    double timestep = 0.01;             // 100 Hz (10ms)
    
    // Middleware settings
    double updateRate = 100.0;          // Hz
    bool enableValidation = true;
    bool enableLogging = false;
    
    // Timing constraints
    double maxStepTime = 0.02;          // Maximum allowed step time (20ms)
    
    // Control
    bool enableControlCommands = true;
};

// Statistics for monitoring engine performance
struct EngineStatistics {
    uint64_t totalCycles = 0;
    uint64_t commandsReceived = 0;
    uint64_t commandsApplied = 0;
    uint64_t commandsRejected = 0;
    
    double averageStepTime = 0.0;
    double maxStepTime = 0.0;
    double averageLoopTime = 0.0;
    
    double currentSimTime = 0.0;
    double realTimeElapsed = 0.0;
    double realTimeFactor = 0.0;        // sim_time / real_time
};

// Main middleware engine - orchestrates data flow
class MiddlewareEngine {
public:
    MiddlewareEngine();
    explicit MiddlewareEngine(const EngineConfig& config);
    ~MiddlewareEngine();
    
    // Initialize the engine (create simulator, setup IPC)
    bool initialize();
    
    // Run the main loop (blocking)
    void run();
    
    // Stop the engine (can be called from signal handler)
    void stop();
    
    // Check if engine is running
    bool isRunning() const;
    
    // Configuration
    const EngineConfig& getConfig() const { return config_; }
    void setConfig(const EngineConfig& config) { config_ = config; }
    
    // Statistics
    EngineStatistics getStatistics() const;
    
    // Component management
    void setSimulator(std::unique_ptr<simulator::ISimulator> sim);
    void addIPCAdapter(std::unique_ptr<ipc::IIPCAdapter> adapter);
    
private:
    // Main loop operations
    void mainLoopIteration();
    void publishPerceptionData();
    void handleControlCommand(const data::ControlCommand& cmd);
    
    // Command validation
    bool validateCommand(const data::ControlCommand& cmd) const;
    
    // Statistics and timing
    void updateStatistics(double loopTime, double stepTime);
    double getCurrentTime() const;
    void sleepUntil(double targetTime);
    
    // Configuration
    EngineConfig config_;
    
    // Components
    std::unique_ptr<simulator::ISimulator> simulator_;
    std::vector<std::unique_ptr<ipc::IIPCAdapter>> ipcAdapters_;
    
    // State
    std::atomic<bool> running_;
    bool initialized_;
    
    // Timing
    double startTime_;
    double lastStepTime_;
    
    // Statistics
    EngineStatistics stats_;
    double totalStepTime_;
    double totalLoopTime_;
};

} // namespace middleware::core
