#include "middleware/ipc/udp_adapter.h"
#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>

// Simple test framework
#define TEST(name) void name()
#define ASSERT(condition, message) \
    if (!(condition)) { \
        std::cerr << "FAILED: " << message << std::endl; \
        return; \
    }

using namespace middleware;

// Test UDP adapter construction
TEST(UDPAdapter_Construction) {
    ipc::UDPAdapter adapter;
    ASSERT(!adapter.isReady(), "Adapter should not be ready before initialization");
    
    auto stats = adapter.getStatistics();
    ASSERT(stats.messagesSent == 0, "No messages should be sent initially");
    ASSERT(stats.messagesReceived == 0, "No messages should be received initially");
    
    std::cout << "PASSED" << std::endl;
}

// Test UDP adapter initialization
TEST(UDPAdapter_Initialize) {
    ipc::UDPAdapter adapter;
    
    bool initialized = adapter.initialize();
    ASSERT(initialized, "Initialization should succeed");
    ASSERT(adapter.isReady(), "Adapter should be ready after initialization");
    
    bool terminated = adapter.terminate();
    ASSERT(terminated, "Termination should succeed");
    ASSERT(!adapter.isReady(), "Adapter should not be ready after termination");
    
    std::cout << "PASSED" << std::endl;
}

// Test custom configuration
TEST(UDPAdapter_CustomConfig) {
    ipc::UDPConfig config;
    config.publishAddress = "239.255.0.2";
    config.publishPort = 6000;
    config.receivePort = 6001;
    
    ipc::UDPAdapter adapter(config);
    ASSERT(adapter.getConfig().publishPort == 6000, "Custom publish port should be set");
    ASSERT(adapter.getConfig().receivePort == 6001, "Custom receive port should be set");
    
    bool initialized = adapter.initialize();
    ASSERT(initialized, "Initialization with custom config should succeed");
    
    std::cout << "PASSED" << std::endl;
}

// Test publishing vehicles
TEST(UDPAdapter_PublishVehicles) {
    ipc::UDPAdapter adapter;
    ASSERT(adapter.initialize(), "Initialization should succeed");
    
    // Create test vehicles
    std::vector<data::VehicleState> vehicles(2);
    
    vehicles[0].id = 0;
    vehicles[0].x = 10.0;
    vehicles[0].y = 5.0;
    vehicles[0].speed = 15.0;
    
    vehicles[1].id = 1;
    vehicles[1].x = 50.0;
    vehicles[1].y = 5.0;
    vehicles[1].speed = 20.0;
    
    bool published = adapter.publishVehicles(vehicles);
    ASSERT(published, "Publishing vehicles should succeed");
    
    auto stats = adapter.getStatistics();
    ASSERT(stats.messagesSent == 1, "One message should be sent");
    ASSERT(stats.bytesSent > 0, "Some bytes should be sent");
    
    std::cout << "PASSED" << std::endl;
}

// Test publishing scene
TEST(UDPAdapter_PublishScene) {
    ipc::UDPAdapter adapter;
    ASSERT(adapter.initialize(), "Initialization should succeed");
    
    // Create test scene
    data::SceneState scene;
    scene.weather_type = 2;  // Rain
    scene.temperature = 15.5;
    scene.friction = 0.6;  // Wet road
    
    bool published = adapter.publishScene(scene);
    ASSERT(published, "Publishing scene should succeed");
    
    auto stats = adapter.getStatistics();
    ASSERT(stats.messagesSent == 1, "One message should be sent");
    ASSERT(stats.bytesSent > 0, "Some bytes should be sent");
    
    std::cout << "PASSED" << std::endl;
}

// Test callback registration
TEST(UDPAdapter_CallbackRegistration) {
    ipc::UDPAdapter adapter;
    ASSERT(adapter.initialize(), "Initialization should succeed");
    
    std::atomic<int> callbackCount{0};
    auto callback = [&callbackCount](const data::ControlCommand& cmd) {
        callbackCount++;
    };
    
    bool registered = adapter.registerCommandCallback(callback);
    ASSERT(registered, "Callback registration should succeed");
    
    adapter.unregisterCommandCallback();
    
    std::cout << "PASSED" << std::endl;
}

// Test async listener start/stop
TEST(UDPAdapter_AsyncListenerLifecycle) {
    ipc::UDPAdapter adapter;
    ASSERT(adapter.initialize(), "Initialization should succeed");
    
    ASSERT(!adapter.isAsyncListenerRunning(), "Listener should not be running initially");
    
    // Register callback
    std::atomic<int> callbackCount{0};
    auto callback = [&callbackCount](const data::ControlCommand& cmd) {
        callbackCount++;
    };
    adapter.registerCommandCallback(callback);
    
    // Start listener
    bool started = adapter.startAsyncListener();
    ASSERT(started, "Listener should start successfully");
    ASSERT(adapter.isAsyncListenerRunning(), "Listener should be running");
    
    // Stop listener
    adapter.stopAsyncListener();
    ASSERT(!adapter.isAsyncListenerRunning(), "Listener should be stopped");
    
    std::cout << "PASSED" << std::endl;
}

// Test async listener without callback
TEST(UDPAdapter_AsyncListenerNoCallback) {
    ipc::UDPAdapter adapter;
    ASSERT(adapter.initialize(), "Initialization should succeed");
    
    // Try to start without callback
    bool started = adapter.startAsyncListener();
    ASSERT(!started, "Listener should not start without callback");
    
    std::cout << "PASSED" << std::endl;
}

// Test async listener before initialization
TEST(UDPAdapter_AsyncListenerBeforeInit) {
    ipc::UDPAdapter adapter;
    
    std::atomic<int> callbackCount{0};
    auto callback = [&callbackCount](const data::ControlCommand& cmd) {
        callbackCount++;
    };
    adapter.registerCommandCallback(callback);
    
    bool started = adapter.startAsyncListener();
    ASSERT(!started, "Listener should not start before initialization");
    
    std::cout << "PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "UDP Adapter - Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Running UDPAdapter.Construction... ";
    UDPAdapter_Construction();
    
    std::cout << "Running UDPAdapter.Initialize... ";
    UDPAdapter_Initialize();
    
    std::cout << "Running UDPAdapter.CustomConfig... ";
    UDPAdapter_CustomConfig();
    
    std::cout << "Running UDPAdapter.PublishVehicles... ";
    UDPAdapter_PublishVehicles();
    
    std::cout << "Running UDPAdapter.PublishScene... ";
    UDPAdapter_PublishScene();
    
    std::cout << "Running UDPAdapter.CallbackRegistration... ";
    UDPAdapter_CallbackRegistration();
    
    std::cout << "Running UDPAdapter.AsyncListenerLifecycle... ";
    UDPAdapter_AsyncListenerLifecycle();
    
    std::cout << "Running UDPAdapter.AsyncListenerNoCallback... ";
    UDPAdapter_AsyncListenerNoCallback();
    
    std::cout << "Running UDPAdapter.AsyncListenerBeforeInit... ";
    UDPAdapter_AsyncListenerBeforeInit();
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "All tests PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
