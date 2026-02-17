#include "middleware/core/middleware_engine.h"
#include "middleware/simulator/mock_simulator.h"
#include "middleware/ipc/udp_adapter.h"
#include <iostream>
#include <csignal>
#include <memory>

using namespace middleware;

// Global engine pointer for signal handler
core::MiddlewareEngine* g_engine = nullptr;

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
    if (g_engine) {
        g_engine->stop();
    }
}

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "Middleware Example - Simple Demo" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // Configure engine
    core::EngineConfig config;
    config.timestep = 0.01;          // 100 Hz
    config.updateRate = 100.0;
    config.enableValidation = true;
    config.enableLogging = true;
    config.enableControlCommands = true;
    
    // Create engine
    core::MiddlewareEngine engine(config);
    g_engine = &engine;
    
    // Setup signal handler for Ctrl+C
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Create and configure simulator
    simulator::MockSimulator::Config simConfig;
    simConfig.numVehicles = 2;
    simConfig.initialEgoSpeed = 15.0;    // 15 m/s
    simConfig.initialLeadSpeed = 20.0;   // 20 m/s
    simConfig.leadDistance = 50.0;       // 50m ahead
    
    auto sim = std::make_unique<simulator::MockSimulator>(simConfig);
    engine.setSimulator(std::move(sim));
    
    // Create and configure IPC adapter
    ipc::UDPConfig ipcConfig;
    ipcConfig.publishAddress = "239.255.0.1";  // Multicast
    ipcConfig.publishPort = 48198;
    ipcConfig.receiveAddress = "0.0.0.0";
    ipcConfig.receivePort = 53995;
    ipcConfig.enableMulticast = true;
    
    auto adapter = std::make_unique<ipc::UDPAdapter>(ipcConfig);
    engine.addIPCAdapter(std::move(adapter));
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Simulator: MockSimulator" << std::endl;
    std::cout << "  Timestep: " << config.timestep << " s (" 
              << (1.0 / config.timestep) << " Hz)" << std::endl;
    std::cout << "  IPC: UDP Multicast" << std::endl;
    std::cout << "    Publish: " << ipcConfig.publishAddress << ":" 
              << ipcConfig.publishPort << std::endl;
    std::cout << "    Receive: " << ipcConfig.receiveAddress << ":" 
              << ipcConfig.receivePort << std::endl;
    std::cout << std::endl;
    
    // Initialize
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return 1;
    }
    
    std::cout << "Press Ctrl+C to stop..." << std::endl;
    std::cout << std::endl;
    
    // Run engine (blocking)
    engine.run();
    
    // Print final statistics
    auto stats = engine.getStatistics();
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Final Statistics:" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Total Cycles: " << stats.totalCycles << std::endl;
    std::cout << "  Simulation Time: " << stats.currentSimTime << " s" << std::endl;
    std::cout << "  Real Time: " << stats.realTimeElapsed << " s" << std::endl;
    std::cout << "  Real-Time Factor: " << stats.realTimeFactor << "x" << std::endl;
    std::cout << "  Avg Step Time: " << (stats.averageStepTime * 1000.0) << " ms" << std::endl;
    std::cout << "  Max Step Time: " << (stats.maxStepTime * 1000.0) << " ms" << std::endl;
    std::cout << "  Avg Loop Time: " << (stats.averageLoopTime * 1000.0) << " ms" << std::endl;
    std::cout << "  Commands Received: " << stats.commandsReceived << std::endl;
    std::cout << "  Commands Applied: " << stats.commandsApplied << std::endl;
    std::cout << "  Commands Rejected: " << stats.commandsRejected << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
