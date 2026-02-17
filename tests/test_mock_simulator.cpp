#include "middleware/simulator/mock_simulator.h"
#include <iostream>
#include <cassert>
#include <cmath>

// Simple test framework
#define EXPECT_EQ(a, b) assert((a) == (b))
#define EXPECT_NE(a, b) assert((a) != (b))
#define EXPECT_TRUE(a) assert(a)
#define EXPECT_FALSE(a) assert(!(a))
#define EXPECT_NEAR(a, b, tol) assert(std::abs((a) - (b)) < (tol))
#define TEST(suite, name) void suite##_##name()
#define RUN_TEST(suite, name) do { \
    std::cout << "Running " #suite "." #name "..." << std::flush; \
    suite##_##name(); \
    std::cout << " PASSED" << std::endl; \
} while(0)

using namespace middleware::simulator;
using namespace middleware::data;

// ============================================================================
// MockSimulator Basic Tests
// ============================================================================

TEST(MockSimulator, Construction) {
    MockSimulator sim;
    EXPECT_FALSE(sim.isRunning());
    EXPECT_EQ(sim.getSimulationTime(), 0.0);
}

TEST(MockSimulator, InitializeDefault) {
    MockSimulator sim;
    EXPECT_TRUE(sim.initialize());
    EXPECT_TRUE(sim.isRunning());
    
    auto vehicles = sim.getVehicles();
    EXPECT_EQ(vehicles.size(), 2);  // Default: ego + 1 lead vehicle
}

TEST(MockSimulator, InitializeCustomConfig) {
    MockSimulator::Config config;
    config.numVehicles = 5;
    config.initialEgoSpeed = 10.0;
    config.initialLeadSpeed = 15.0;
    config.leadDistance = 30.0;
    
    MockSimulator sim(config);
    EXPECT_TRUE(sim.initialize());
    
    auto vehicles = sim.getVehicles();
    EXPECT_EQ(vehicles.size(), 5);
    
    // Check ego vehicle
    auto ego = sim.getEgoVehicle();
    EXPECT_EQ(ego.id, 0);
    EXPECT_EQ(ego.x, 0.0);
    EXPECT_NEAR(ego.speed, 10.0, 0.01);
    
    // Check first lead vehicle
    EXPECT_NEAR(vehicles[1].x, 30.0, 0.01);
    EXPECT_NEAR(vehicles[1].speed, 15.0, 0.01);
}

TEST(MockSimulator, EgoVehicle) {
    MockSimulator sim;
    sim.initialize();
    
    auto ego = sim.getEgoVehicle();
    EXPECT_EQ(ego.id, 0);
    EXPECT_EQ(ego.control_mode, 1);  // Autonomous
}

TEST(MockSimulator, SceneState) {
    MockSimulator sim;
    sim.initialize();
    
    auto scene = sim.getScene();
    EXPECT_EQ(scene.timestamp, 0.0);
    EXPECT_EQ(scene.num_vehicles, 2);
    EXPECT_NEAR(scene.temperature, 20.0, 0.01);
    EXPECT_NEAR(scene.friction, 0.8, 0.01);
}

// ============================================================================
// Physics and Simulation Tests
// ============================================================================

TEST(MockSimulator, StepForward) {
    MockSimulator sim;
    sim.initialize();
    
    auto initialTime = sim.getSimulationTime();
    auto initialVehicles = sim.getVehicles();
    
    double dt = 0.01;  // 10ms step
    EXPECT_TRUE(sim.step(dt));
    
    EXPECT_NEAR(sim.getSimulationTime(), initialTime + dt, 1e-9);
    
    // Vehicles should have moved
    auto vehicles = sim.getVehicles();
    EXPECT_NE(vehicles[0].x, initialVehicles[0].x);
}

TEST(MockSimulator, VehicleMotion) {
    MockSimulator sim;
    sim.initialize();
    
    auto ego = sim.getEgoVehicle();
    double initialX = ego.x;
    double initialSpeed = ego.speed;
    
    // Step 1 second
    for (int i = 0; i < 100; ++i) {
        sim.step(0.01);
    }
    
    ego = sim.getEgoVehicle();
    
    // Vehicle should have moved forward
    EXPECT_TRUE(ego.x > initialX);
    
    // Distance = speed * time (approximately, for constant speed)
    double expectedDistance = initialSpeed * 1.0;
    EXPECT_NEAR(ego.x, initialX + expectedDistance, 0.5);  // Some tolerance
}

TEST(MockSimulator, MultipleVehicles) {
    MockSimulator::Config config;
    config.numVehicles = 3;
    
    MockSimulator sim(config);
    sim.initialize();
    
    auto vehicles = sim.getVehicles();
    EXPECT_EQ(vehicles.size(), 3);
    
    // All should have unique IDs
    EXPECT_EQ(vehicles[0].id, 0);
    EXPECT_EQ(vehicles[1].id, 1);
    EXPECT_EQ(vehicles[2].id, 2);
    
    // All should be moving
    sim.step(0.1);
    auto newVehicles = sim.getVehicles();
    
    for (size_t i = 0; i < vehicles.size(); ++i) {
        EXPECT_NE(newVehicles[i].x, vehicles[i].x);
    }
}

// ============================================================================
// Control Command Tests
// ============================================================================

TEST(MockSimulator, SpeedControl) {
    MockSimulator sim;
    sim.initialize();
    
    auto ego = sim.getEgoVehicle();
    double initialSpeed = ego.speed;
    
    // Create speed control command
    ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.mode = ControlCommand::Mode::SPEED;
    cmd.speed.targetSpeed = 25.0;  // Target 25 m/s
    cmd.speed.maxAcceleration = 2.0;
    
    EXPECT_TRUE(sim.applyControl(cmd));
    
    // Step simulation for 5 seconds
    for (int i = 0; i < 500; ++i) {
        sim.step(0.01);
    }
    
    ego = sim.getEgoVehicle();
    
    // Speed should have increased towards target
    EXPECT_TRUE(ego.speed > initialSpeed);
    EXPECT_NEAR(ego.speed, 25.0, 1.0);  // Should be close to target
}

TEST(MockSimulator, AccelerationControl) {
    MockSimulator sim;
    sim.initialize();
    
    auto ego = sim.getEgoVehicle();
    double initialSpeed = ego.speed;
    
    // Create acceleration control command
    ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.mode = ControlCommand::Mode::ACCELERATION;
    cmd.acceleration.targetAcceleration = 3.0;  // 3 m/s²
    
    EXPECT_TRUE(sim.applyControl(cmd));
    
    // Step for 1 second
    for (int i = 0; i < 100; ++i) {
        sim.step(0.01);
    }
    
    ego = sim.getEgoVehicle();
    
    // Speed should have increased by ~3 m/s
    EXPECT_NEAR(ego.speed, initialSpeed + 3.0, 0.5);
}

TEST(MockSimulator, PositionControl) {
    MockSimulator sim;
    sim.initialize();
    
    // Teleport vehicle to new position
    ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.mode = ControlCommand::Mode::POSITION;
    cmd.position.x = 100.0;
    cmd.position.y = 50.0;
    cmd.position.z = 5.0;
    cmd.position.heading = 1.57;  // 90 degrees
    
    EXPECT_TRUE(sim.applyControl(cmd));
    
    auto ego = sim.getEgoVehicle();
    
    EXPECT_NEAR(ego.x, 100.0, 0.01);
    EXPECT_NEAR(ego.y, 50.0, 0.01);
    EXPECT_NEAR(ego.z, 5.0, 0.01);
    EXPECT_NEAR(ego.heading, 1.57, 0.01);
}

TEST(MockSimulator, ThrottleBrakeControl) {
    MockSimulator sim;
    sim.initialize();
    
    // Apply throttle
    ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.mode = ControlCommand::Mode::THROTTLE_BRAKE;
    cmd.throttleBrake.throttle = 0.8;
    cmd.throttleBrake.brake = 0.0;
    
    EXPECT_TRUE(sim.applyControl(cmd));
    
    auto ego = sim.getEgoVehicle();
    EXPECT_NEAR(ego.throttle, 0.8, 0.01);
    EXPECT_TRUE(ego.acceleration > 0);  // Should be accelerating
}

TEST(MockSimulator, ControlInvalidVehicle) {
    MockSimulator sim;
    sim.initialize();
    
    ControlCommand cmd;
    cmd.vehicleId = 999;  // Non-existent vehicle
    cmd.mode = ControlCommand::Mode::SPEED;
    cmd.speed.targetSpeed = 20.0;
    
    EXPECT_FALSE(sim.applyControl(cmd));  // Should fail
}

TEST(MockSimulator, ControlMultipleVehicles) {
    MockSimulator::Config config;
    config.numVehicles = 3;
    
    MockSimulator sim(config);
    sim.initialize();
    
    // Control vehicle 1
    ControlCommand cmd1;
    cmd1.vehicleId = 1;
    cmd1.mode = ControlCommand::Mode::SPEED;
    cmd1.speed.targetSpeed = 30.0;
    
    EXPECT_TRUE(sim.applyControl(cmd1));
    
    // Control vehicle 2
    ControlCommand cmd2;
    cmd2.vehicleId = 2;
    cmd2.mode = ControlCommand::Mode::SPEED;
    cmd2.speed.targetSpeed = 10.0;
    
    EXPECT_TRUE(sim.applyControl(cmd2));
    
    // Both commands should be applied
    sim.step(0.01);
}

// ============================================================================
// Reset and Lifecycle Tests
// ============================================================================

TEST(MockSimulator, Reset) {
    MockSimulator sim;
    sim.initialize();
    
    // Step simulation
    for (int i = 0; i < 100; ++i) {
        sim.step(0.01);
    }
    
    auto timeBeforeReset = sim.getSimulationTime();
    EXPECT_TRUE(timeBeforeReset > 0);
    
    // Reset
    EXPECT_TRUE(sim.reset());
    
    // Should be back to initial state
    EXPECT_EQ(sim.getSimulationTime(), 0.0);
    EXPECT_TRUE(sim.isRunning());
    
    auto ego = sim.getEgoVehicle();
    EXPECT_EQ(ego.x, 0.0);
}

TEST(MockSimulator, Terminate) {
    MockSimulator sim;
    sim.initialize();
    
    EXPECT_TRUE(sim.isRunning());
    
    sim.terminate();
    
    EXPECT_FALSE(sim.isRunning());
}

TEST(MockSimulator, StepAfterTerminate) {
    MockSimulator sim;
    sim.initialize();
    
    sim.terminate();
    
    EXPECT_FALSE(sim.step(0.01));  // Should fail
}

// ============================================================================
// SimulatorFactory Tests
// ============================================================================

TEST(SimulatorFactory, CreateMock) {
    auto sim = SimulatorFactory::create(SimulatorFactory::Type::MOCK);
    
    EXPECT_TRUE(sim != nullptr);
    EXPECT_TRUE(sim->initialize());
    EXPECT_TRUE(sim->isRunning());
}

TEST(SimulatorFactory, CreateFromString) {
    auto sim = SimulatorFactory::create("mock");
    
    EXPECT_TRUE(sim != nullptr);
    EXPECT_TRUE(sim->initialize());
}

TEST(SimulatorFactory, CreateUnimplemented) {
    auto sim = SimulatorFactory::create(SimulatorFactory::Type::ESMINI);
    EXPECT_TRUE(sim == nullptr);  // Not implemented yet
}

// ============================================================================
// Integration Scenario Tests
// ============================================================================

TEST(Scenario, EgoFollowingLead) {
    MockSimulator::Config config;
    config.numVehicles = 2;
    config.initialEgoSpeed = 15.0;   // Ego at 15 m/s
    config.initialLeadSpeed = 20.0;  // Lead at 20 m/s
    config.leadDistance = 50.0;      // 50m apart
    
    MockSimulator sim(config);
    sim.initialize();
    
    auto ego = sim.getEgoVehicle();
    auto vehicles = sim.getVehicles();
    auto lead = vehicles[1];
    
    double initialDistance = lead.x - ego.x;
    EXPECT_NEAR(initialDistance, 50.0, 0.01);
    
    // Lead is faster, so gap should increase
    for (int i = 0; i < 100; ++i) {
        sim.step(0.01);
    }
    
    ego = sim.getEgoVehicle();
    vehicles = sim.getVehicles();
    lead = vehicles[1];
    
    double newDistance = lead.x - ego.x;
    EXPECT_TRUE(newDistance > initialDistance);  // Gap increased
}

TEST(Scenario, ACCLikeControl) {
    // Simulate ACC: ego accelerates to match lead speed
    MockSimulator::Config config;
    config.numVehicles = 2;
    config.initialEgoSpeed = 15.0;
    config.initialLeadSpeed = 20.0;
    config.leadDistance = 50.0;
    
    MockSimulator sim(config);
    sim.initialize();
    
    // Apply speed control to match lead
    ControlCommand cmd;
    cmd.vehicleId = 0;
    cmd.mode = ControlCommand::Mode::SPEED;
    cmd.speed.targetSpeed = 20.0;  // Match lead speed
    cmd.speed.maxAcceleration = 2.0;
    
    sim.applyControl(cmd);
    
    // Simulate for 5 seconds
    for (int i = 0; i < 500; ++i) {
        sim.step(0.01);
    }
    
    auto ego = sim.getEgoVehicle();
    
    // Ego should now be close to lead speed
    EXPECT_NEAR(ego.speed, 20.0, 1.0);
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "MockSimulator - Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // Basic tests
    RUN_TEST(MockSimulator, Construction);
    RUN_TEST(MockSimulator, InitializeDefault);
    RUN_TEST(MockSimulator, InitializeCustomConfig);
    RUN_TEST(MockSimulator, EgoVehicle);
    RUN_TEST(MockSimulator, SceneState);
    
    // Physics tests
    RUN_TEST(MockSimulator, StepForward);
    RUN_TEST(MockSimulator, VehicleMotion);
    RUN_TEST(MockSimulator, MultipleVehicles);
    
    // Control tests
    RUN_TEST(MockSimulator, SpeedControl);
    RUN_TEST(MockSimulator, AccelerationControl);
    RUN_TEST(MockSimulator, PositionControl);
    RUN_TEST(MockSimulator, ThrottleBrakeControl);
    RUN_TEST(MockSimulator, ControlInvalidVehicle);
    RUN_TEST(MockSimulator, ControlMultipleVehicles);
    
    // Lifecycle tests
    RUN_TEST(MockSimulator, Reset);
    RUN_TEST(MockSimulator, Terminate);
    RUN_TEST(MockSimulator, StepAfterTerminate);
    
    // Factory tests
    RUN_TEST(SimulatorFactory, CreateMock);
    RUN_TEST(SimulatorFactory, CreateFromString);
    RUN_TEST(SimulatorFactory, CreateUnimplemented);
    
    // Integration scenarios
    RUN_TEST(Scenario, EgoFollowingLead);
    RUN_TEST(Scenario, ACCLikeControl);
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "All tests PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
