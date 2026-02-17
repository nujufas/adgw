#include "middleware/core/middleware_engine.h"
#include "middleware/simulator/esmini_adapter.h"
#include "middleware/ipc/udp_adapter.h"
#include <iostream>
#include <csignal>
#include <memory>
#include <string>

using namespace middleware;

// Global engine pointer for signal handler
core::MiddlewareEngine* g_engine = nullptr;

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
    if (g_engine) {
        g_engine->stop();
    }
}

void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " <scenario_file.xosc> [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --headless           Run without graphics (default)" << std::endl;
    std::cout << "  --viewer             Run with esmini viewer" << std::endl;
    std::cout << "  --help               Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << progName << " scenarios/acc_test.xosc" << std::endl;
    std::cout << "  " << progName << " scenarios/acc_test.xosc --viewer" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "Middleware Example - esmini Demo" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // Parse command line arguments
    if (argc < 2) {
        std::cerr << "Error: Missing scenario file" << std::endl;
        std::cerr << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    
    std::string scenarioFile = argv[1];
    bool headless = true;
    
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--viewer") {
            headless = false;
        } else if (arg == "--headless") {
            headless = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
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
    
    // Create and configure esmini simulator
    try {
        auto sim = std::make_unique<simulator::EsminiAdapter>(
            scenarioFile,
            headless,
            true  // disable built-in controllers
        );
        engine.setSimulator(std::move(sim));
    } catch (const std::exception& e) {
        std::cerr << "Failed to create esmini adapter: " << e.what() << std::endl;
        return 1;
    }
    
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
    std::cout << "  Simulator: esmini" << std::endl;
    std::cout << "  Scenario: " << scenarioFile << std::endl;
    std::cout << "  Headless: " << (headless ? "yes" : "no") << std::endl;
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
    std::cout << "========================================" << std::endl;
    
    return 0;
}
