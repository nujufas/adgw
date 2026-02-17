#include "middleware/core/middleware_engine.h"
#include "middleware/simulator/mock_simulator.h"
#include "middleware/ipc/udp_adapter.h"

#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <atomic>

// Simple test framework
#define EXPECT_EQ(a, b) assert((a) == (b))
#define EXPECT_NE(a, b) assert((a) != (b))
#define EXPECT_TRUE(a) assert(a)
#define EXPECT_FALSE(a) assert(!(a))
#define EXPECT_GT(a, b) assert((a) > (b))
#define EXPECT_LT(a, b) assert((a) < (b))
#define EXPECT_GE(a, b) assert((a) >= (b))
#define EXPECT_LE(a, b) assert((a) <= (b))
#define EXPECT_NEAR(a, b, tol) assert(std::abs((a) - (b)) <= (tol))
#define ASSERT_TRUE(a) assert(a)
#define ASSERT_FALSE(a) assert(!(a))
#define ASSERT_GE(a, b) assert((a) >= (b))
#define ASSERT_EQ(a, b) assert((a) == (b))
#define TEST(suite, name) void suite##_##name()
#define RUN_TEST(suite, name) do { \
    std::cout << "Running " #suite "." #name "..." << std::flush; \
    suite##_##name(); \
    std::cout << " PASSED" << std::endl; \
} while(0)

using namespace middleware;

// ============================================================================
// Helper class for test setup
// ============================================================================

class IntegrationTestFixture {
public:
    IntegrationTestFixture() {
        // Create simulator
        simulator::MockSimulator::Config simConfig;
        simConfig.numVehicles = 2;
        simConfig.initialEgoSpeed = 15.0;
        simConfig.initialLeadSpeed = 20.0;
        simConfig.leadDistance = 50.0;
        simulator_ = std::make_unique<simulator::MockSimulator>(simConfig);
        
        // Create IPC adapter
        ipc::UDPAdapter::Config ipcConfig;
        ipcConfig.multicastGroup = "239.255.0.1";
        ipcConfig.publishPort = 48198;
        ipcConfig.receivePort = 53995;
        ipcConfig.enableMulticast = true;
        ipcAdapter_ = std::make_unique<ipc::UDPAdapter>(ipcConfig);
        
        // Create engine
        core::MiddlewareEngine::EngineConfig engineConfig;
        engineConfig.timestep = 0.01;  // 100 Hz
        engineConfig.enableCommandValidation = true;
        engineConfig.enableControlCommands = true;
        engine_ = std::make_unique<core::MiddlewareEngine>(engineConfig);
    }
    
    ~IntegrationTestFixture() {
        if (engine_) {
            engine_->stop();
        }
    }
    
    std::unique_ptr<simulator::MockSimulator> simulator_;
    std::unique_ptr<ipc::UDPAdapter> ipcAdapter_;
    std::unique_ptr<core::MiddlewareEngine> engine_;
};

// ============================================================================
// Test 1: Basic Communication
// ============================================================================

TEST(IntegrationTest, BasicCommunication) {
    IntegrationTestFixture fixture;
    
    // Initialize engine
    ASSERT_TRUE(fixture.engine_->initialize(fixture.simulator_.get(), {fixture.ipcAdapter_.get()}));
    
    // Run for a short time
    std::thread engineThread([this]() {
        engine_->run();
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    engine_->stop();
    engineThread.join();
    
    // Get statistics
    auto stats = engine_->getStatistics();
    
    // Should have run at least 5 cycles (100ms at 100 Hz = 10 cycles)
    EXPECT_GE(stats.totalCycles, 5);
    
    // Should have published data
    auto ipcStats = ipcAdapter_->getStatistics();
    EXPECT_GT(ipcStats.messagesSent, 0);
}

// ============================================================================
// Test 2: Closed-Loop Control
// ============================================================================

TEST_F(IntegrationTest, ClosedLoopControl) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Get initial ego speed
    auto initialVehicles = simulator_->getVehicles();
    ASSERT_GE(initialVehicles.size(), 1);
    double initialSpeed = initialVehicles[0].speed;
    
    // Send speed command
    data::ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.timestamp = simulator_->getCurrentTime();
    cmd.sequenceNumber = 1;
    cmd.mode = data::ControlCommand::Mode::SPEED;
    cmd.speed.targetSpeed = 25.0;  // Faster than initial
    cmd.speed.maxAcceleration = 3.0;
    
    ASSERT_TRUE(simulator_->applyControl(cmd));
    
    // Step simulator
    for (int i = 0; i < 10; i++) {
        ASSERT_TRUE(simulator_->step(0.01));
    }
    
    // Get new speed
    auto newVehicles = simulator_->getVehicles();
    ASSERT_GE(newVehicles.size(), 1);
    double newSpeed = newVehicles[0].speed;
    
    // Speed should have changed toward target
    EXPECT_NE(newSpeed, initialSpeed);
}

// ============================================================================
// Test 3: Command Validation
// ============================================================================

TEST_F(IntegrationTest, CommandValidation) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    auto vehicles = simulator_->getVehicles();
    ASSERT_GE(vehicles.size(), 1);
    
    // Test 1: Valid command should work
    data::ControlCommand validCmd;
    validCmd.vehicleId = vehicles[0].id;
    validCmd.timestamp = simulator_->getCurrentTime();
    validCmd.mode = data::ControlCommand::Mode::SPEED;
    validCmd.speed.targetSpeed = 20.0;
    
    EXPECT_TRUE(simulator_->applyControl(validCmd));
    
    // Test 2: Invalid vehicle ID should be rejected
    data::ControlCommand invalidVehicle;
    invalidVehicle.vehicleId = 999;
    invalidVehicle.timestamp = simulator_->getCurrentTime();
    invalidVehicle.mode = data::ControlCommand::Mode::SPEED;
    invalidVehicle.speed.targetSpeed = 20.0;
    
    EXPECT_FALSE(simulator_->applyControl(invalidVehicle));
    
    // Test 3: Stale timestamp (if validation enabled)
    data::ControlCommand staleCmd;
    staleCmd.vehicleId = vehicles[0].id;
    staleCmd.timestamp = simulator_->getCurrentTime() - 10.0;  // 10 seconds old
    staleCmd.mode = data::ControlCommand::Mode::SPEED;
    staleCmd.speed.targetSpeed = 20.0;
    
    // Stale command might be rejected by middleware, but MockSimulator accepts all
    // This is OK - validation happens at middleware level
}

// ============================================================================
// Test 4: Multi-Cycle Stability
// ============================================================================

TEST_F(IntegrationTest, MultiCycleStability) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Run for 1 second (100 cycles at 100 Hz)
    std::thread engineThread([this]() {
        engine_->run();
    });
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    engine_->stop();
    engineThread.join();
    
    // Get statistics
    auto stats = engine_->getStatistics();
    
    // Should have run approximately 100 cycles
    EXPECT_GE(stats.totalCycles, 90);  // Allow some tolerance
    EXPECT_LE(stats.totalCycles, 110);
    
    // Real-time factor should be close to 1.0
    EXPECT_GT(stats.realTimeFactor, 0.8);
    EXPECT_LT(stats.realTimeFactor, 1.2);
    
    // Should have published data every cycle
    auto ipcStats = ipcAdapter_->getStatistics();
    EXPECT_GE(ipcStats.messagesSent, stats.totalCycles);
}

// ============================================================================
// Test 5: Multiple Vehicles
// ============================================================================

TEST_F(IntegrationTest, MultipleVehicles) {
    // Already configured with 2 vehicles in SetUp
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    auto vehicles = simulator_->getVehicles();
    ASSERT_EQ(vehicles.size(), 2);
    
    // Verify vehicle properties
    EXPECT_EQ(vehicles[0].id, 0);  // Ego
    EXPECT_EQ(vehicles[1].id, 1);  // Lead
    
    // Verify initial positions
    EXPECT_NEAR(vehicles[0].x, 0.0, 0.1);
    EXPECT_NEAR(vehicles[1].x, 50.0, 0.1);  // Lead is 50m ahead
    
    // Verify speeds
    EXPECT_NEAR(vehicles[0].speed, 15.0, 0.1);
    EXPECT_NEAR(vehicles[1].speed, 20.0, 0.1);
}

// ============================================================================
// Test 6: Real-Time Performance
// ============================================================================

TEST_F(IntegrationTest, RealTimePerformance) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Run for 500ms
    auto startTime = std::chrono::steady_clock::now();
    
    std::thread engineThread([this]() {
        engine_->run();
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    engine_->stop();
    engineThread.join();
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(endTime - startTime).count();
    
    // Get statistics
    auto stats = engine_->getStatistics();
    
    // Calculate achieved rate
    double achievedRate = stats.totalCycles / elapsed;
    
    // Should achieve close to 100 Hz
    EXPECT_GT(achievedRate, 80.0);   // At least 80 Hz
    EXPECT_LT(achievedRate, 120.0);  // Not much more than 100 Hz
    
    // Average step time should be reasonable
    EXPECT_LT(stats.averageStepTime, 5.0);  // Less than 5ms
}

// ============================================================================
// Test 7: IPC Statistics
// ============================================================================

TEST_F(IntegrationTest, IPCStatistics) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Run for 200ms
    std::thread engineThread([this]() {
        engine_->run();
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    engine_->stop();
    engineThread.join();
    
    // Get IPC statistics
    auto ipcStats = ipcAdapter_->getStatistics();
    
    // Should have sent messages
    EXPECT_GT(ipcStats.messagesSent, 0);
    EXPECT_GT(ipcStats.bytesSent, 0);
    
    // Should have no errors (local UDP should be reliable)
    EXPECT_EQ(ipcStats.sendErrors, 0);
    
    // Bytes sent should be reasonable (each message ~400 bytes)
    size_t expectedBytes = ipcStats.messagesSent * 300;  // At least 300 bytes per message
    EXPECT_GT(ipcStats.bytesSent, expectedBytes);
}

// ============================================================================
// Test 8: Graceful Shutdown
// ============================================================================

TEST_F(IntegrationTest, GracefulShutdown) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Start engine
    std::thread engineThread([this]() {
        engine_->run();
    });
    
    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Stop should work cleanly
    EXPECT_NO_THROW(engine_->stop());
    
    // Thread should join without hanging
    ASSERT_NO_THROW(engineThread.join());
    
    // Should have some statistics
    auto stats = engine_->getStatistics();
    EXPECT_GT(stats.totalCycles, 0);
}

// ============================================================================
// Test 9: Command Application Rate
// ============================================================================

TEST_F(IntegrationTest, CommandApplicationRate) {
    // Initialize engine with command callback
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    // Send multiple commands rapidly
    std::atomic<int> commandsApplied{0};
    
    auto vehicles = simulator_->getVehicles();
    ASSERT_GE(vehicles.size(), 1);
    
    for (int i = 0; i < 10; i++) {
        data::ControlCommand cmd;
        cmd.vehicleId = vehicles[0].id;
        cmd.timestamp = simulator_->getCurrentTime() + i * 0.01;
        cmd.mode = data::ControlCommand::Mode::SPEED;
        cmd.speed.targetSpeed = 15.0 + i;
        
        if (simulator_->applyControl(cmd)) {
            commandsApplied++;
        }
    }
    
    // All valid commands should be applied
    EXPECT_EQ(commandsApplied, 10);
}

// ============================================================================
// Test 10: Simulation Time Progression
// ============================================================================

TEST_F(IntegrationTest, SimulationTimeProgression) {
    // Initialize engine
    ASSERT_TRUE(engine_->initialize(simulator_.get(), {ipcAdapter_.get()}));
    
    double initialTime = simulator_->getCurrentTime();
    
    // Step simulator 100 times with 0.01s timestep
    for (int i = 0; i < 100; i++) {
        ASSERT_TRUE(simulator_->step(0.01));
    }
    
    double finalTime = simulator_->getCurrentTime();
    
    // Should have advanced by 1.0 second
    EXPECT_NEAR(finalTime - initialTime, 1.0, 0.01);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
