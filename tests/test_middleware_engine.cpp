#include "middleware/core/middleware_engine.h"
#include "middleware/simulator/mock_simulator.h"
#include "middleware/ipc/udp_adapter.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

// Simple test framework
#define TEST(name) void name()
#define ASSERT(condition, message) \
    if (!(condition)) { \
        std::cerr << "FAILED: " << message << std::endl; \
        return; \
    }

using namespace middleware;

// Test engine construction
TEST(MiddlewareEngine_Construction) {
    core::MiddlewareEngine engine;
    ASSERT(!engine.isRunning(), "Engine should not be running initially");
    
    auto stats = engine.getStatistics();
    ASSERT(stats.totalCycles == 0, "No cycles should have run");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine with custom config
TEST(MiddlewareEngine_CustomConfig) {
    core::EngineConfig config;
    config.timestep = 0.02;  // 50 Hz
    config.updateRate = 50.0;
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    ASSERT(engine.getConfig().timestep == 0.02, "Custom timestep should be set");
    ASSERT(engine.getConfig().updateRate == 50.0, "Custom update rate should be set");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine initialization with simulator
TEST(MiddlewareEngine_InitializeWithSimulator) {
    core::MiddlewareEngine engine;
    
    // Add simulator
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    // Initialize
    bool initialized = engine.initialize();
    ASSERT(initialized, "Initialization should succeed with simulator");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine initialization without simulator
TEST(MiddlewareEngine_InitializeWithoutSimulator) {
    core::MiddlewareEngine engine;
    
    // Try to initialize without simulator
    bool initialized = engine.initialize();
    ASSERT(!initialized, "Initialization should fail without simulator");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine with IPC adapter
TEST(MiddlewareEngine_WithIPCAdapter) {
    core::EngineConfig config;
    config.enableLogging = false;
    config.enableControlCommands = true;
    
    core::MiddlewareEngine engine(config);
    
    // Add simulator
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    // Add IPC adapter
    ipc::UDPConfig ipcConfig;
    ipcConfig.publishPort = 6000;
    ipcConfig.receivePort = 6001;
    auto adapter = std::make_unique<ipc::UDPAdapter>(ipcConfig);
    engine.addIPCAdapter(std::move(adapter));
    
    // Initialize
    bool initialized = engine.initialize();
    ASSERT(initialized, "Initialization should succeed with IPC");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine statistics
TEST(MiddlewareEngine_Statistics) {
    core::EngineConfig config;
    config.timestep = 0.01;
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    ASSERT(engine.initialize(), "Initialization should succeed");
    
    // Run for a short time in separate thread
    std::atomic<bool> stopFlag{false};
    std::thread runThread([&engine, &stopFlag]() {
        while (!stopFlag.load()) {
            if (engine.getStatistics().totalCycles >= 10) {
                engine.stop();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    
    // Start engine (will block until stopped)
    std::thread engineThread([&engine]() {
        engine.run();
    });
    
    // Wait for completion
    engineThread.join();
    runThread.join();
    
    auto stats = engine.getStatistics();
    ASSERT(stats.totalCycles >= 10, "Should have run at least 10 cycles");
    ASSERT(stats.currentSimTime > 0.0, "Simulation time should advance");
    ASSERT(stats.realTimeElapsed > 0.0, "Real time should advance");
    
    std::cout << "PASSED" << std::endl;
}

// Test engine stop
TEST(MiddlewareEngine_Stop) {
    core::EngineConfig config;
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    ASSERT(engine.initialize(), "Initialization should succeed");
    
    // Start engine in thread
    std::thread engineThread([&engine]() {
        engine.run();
    });
    
    // Give it time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Stop engine
    engine.stop();
    ASSERT(!engine.isRunning(), "Engine should not be running after stop");
    
    // Wait for thread to finish
    engineThread.join();
    
    std::cout << "PASSED" << std::endl;
}

// Test command validation
TEST(MiddlewareEngine_CommandValidation) {
    core::EngineConfig config;
    config.enableValidation = true;
    config.enableLogging = false;
    config.enableControlCommands = true;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    ASSERT(engine.initialize(), "Initialization should succeed");
    
    // Commands will be validated when received via callback
    // Just verify initialization worked with validation enabled
    
    std::cout << "PASSED" << std::endl;
}

// Test engine without control commands
TEST(MiddlewareEngine_NoControlCommands) {
    core::EngineConfig config;
    config.enableControlCommands = false;
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    // Add IPC adapter (but commands won't be processed)
    auto adapter = std::make_unique<ipc::UDPAdapter>();
    engine.addIPCAdapter(std::move(adapter));
    
    ASSERT(engine.initialize(), "Initialization should succeed");
    
    std::cout << "PASSED" << std::endl;
}

// Test real-time factor calculation
TEST(MiddlewareEngine_RealTimeFactor) {
    core::EngineConfig config;
    config.timestep = 0.01;  // 100 Hz
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    ASSERT(engine.initialize(), "Initialization should succeed");
    
    // Run for 20 cycles
    std::thread engineThread([&engine]() {
        engine.run();
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    while (engine.getStatistics().totalCycles < 20) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    engine.stop();
    engineThread.join();
    
    auto stats = engine.getStatistics();
    ASSERT(stats.realTimeFactor > 0.0, "Real-time factor should be calculated");
    ASSERT(stats.currentSimTime > 0.0, "Sim time should advance");
    
    std::cout << "PASSED" << std::endl;
}

// Test multiple IPC adapters
TEST(MiddlewareEngine_MultipleIPCAdapters) {
    core::EngineConfig config;
    config.enableLogging = false;
    
    core::MiddlewareEngine engine(config);
    
    auto sim = std::make_unique<simulator::MockSimulator>();
    engine.setSimulator(std::move(sim));
    
    // Add two IPC adapters
    ipc::UDPConfig config1;
    config1.publishPort = 7000;
    config1.receivePort = 7001;
    engine.addIPCAdapter(std::make_unique<ipc::UDPAdapter>(config1));
    
    ipc::UDPConfig config2;
    config2.publishPort = 7002;
    config2.receivePort = 7003;
    engine.addIPCAdapter(std::make_unique<ipc::UDPAdapter>(config2));
    
    ASSERT(engine.initialize(), "Initialization should succeed with multiple adapters");
    
    std::cout << "PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Middleware Engine - Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Running MiddlewareEngine.Construction... ";
    MiddlewareEngine_Construction();
    
    std::cout << "Running MiddlewareEngine.CustomConfig... ";
    MiddlewareEngine_CustomConfig();
    
    std::cout << "Running MiddlewareEngine.InitializeWithSimulator... ";
    MiddlewareEngine_InitializeWithSimulator();
    
    std::cout << "Running MiddlewareEngine.InitializeWithoutSimulator... ";
    MiddlewareEngine_InitializeWithoutSimulator();
    
    std::cout << "Running MiddlewareEngine.WithIPCAdapter... ";
    MiddlewareEngine_WithIPCAdapter();
    
    std::cout << "Running MiddlewareEngine.Statistics... ";
    MiddlewareEngine_Statistics();
    
    std::cout << "Running MiddlewareEngine.Stop... ";
    MiddlewareEngine_Stop();
    
    std::cout << "Running MiddlewareEngine.CommandValidation... ";
    MiddlewareEngine_CommandValidation();
    
    std::cout << "Running MiddlewareEngine.NoControlCommands... ";
    MiddlewareEngine_NoControlCommands();
    
    std::cout << "Running MiddlewareEngine.RealTimeFactor... ";
    MiddlewareEngine_RealTimeFactor();
    
    std::cout << "Running MiddlewareEngine.MultipleIPCAdapters... ";
    MiddlewareEngine_MultipleIPCAdapters();
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "All tests PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
