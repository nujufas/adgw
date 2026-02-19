/**
 * @file accServerMain.cpp
 * @brief Main entry point for ACC Server application
 * 
 * This application provides the ACC Input Service using CommonAPI/SomeIP.
 * It receives vehicle data from the esmini middleware and publishes it via SomeIP.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <CommonAPI/CommonAPI.hpp>
#include "accInputServiceStubImpl.hpp"

// Global flag for graceful shutdown
std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char** argv) {
    std::cout << "=== ACC SomeIP Server Application ===" << std::endl;
    std::cout << "Starting ACC Input Service Provider..." << std::endl;
    std::cout << std::endl;
    
    // Setup signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Get CommonAPI runtime
    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();
    if (!runtime) {
        std::cerr << "ERROR: Could not get CommonAPI runtime" << std::endl;
        std::cerr << "Check COMMONAPI_CONFIG environment variable" << std::endl;
        return 1;
    }
    
    std::cout << "✓ CommonAPI runtime initialized" << std::endl;
    
    // Create service implementation
    std::shared_ptr<accInputServiceStubImpl> service = 
        std::make_shared<accInputServiceStubImpl>();
    
    // Register service with CommonAPI
    const std::string domain = "local";
    const std::string instance = "acc.input.service";
    
    std::cout << "Registering service..." << std::endl;
    std::cout << "  Domain: " << domain << std::endl;
    std::cout << "  Instance: " << instance << std::endl;
    
    bool success = runtime->registerService(domain, instance, service);
    
    if (!success) {
        std::cerr << "ERROR: Could not register service" << std::endl;
        std::cerr << "Possible causes:" << std::endl;
        std::cerr << "  - vsomeip routing manager not running" << std::endl;
        std::cerr << "  - VSOMEIP_CONFIGURATION not set correctly" << std::endl;
        std::cerr << "  - Port already in use" << std::endl;
        return 1;
    }
    
    std::cout << "✓ Service registered successfully" << std::endl;
    std::cout << std::endl;
    std::cout << "Service Configuration:" << std::endl;
    std::cout << "  Service ID: 0x1234" << std::endl;
    std::cout << "  Instance ID: 0x0001" << std::endl;
    std::cout << "  Port: 30501" << std::endl;
    std::cout << std::endl;
    std::cout << "Service is running. Waiting for middleware connections..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << std::endl;
    
    // Initialize with reasonable defaults
    service->setSystemPowerMode(1.0);  // System ON
    service->setACCCommand(1.0);       // ACC ready
    
    // Main loop - keep service running
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << std::endl;
    std::cout << "Unregistering service..." << std::endl;
    runtime->unregisterService(domain, accInputServiceInterfaceStubDefault::StubInterface::getInterface(), instance);
    
    std::cout << "✓ Service stopped cleanly" << std::endl;
    
    return 0;
}
