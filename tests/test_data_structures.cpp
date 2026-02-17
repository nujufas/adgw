#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include "middleware/data/sensor_data.h"
#include "middleware/utils/serialization.h"

#include <iostream>
#include <cassert>
#include <cmath>

// Simple test framework (if GTest not available)
#ifndef GTEST_FOUND
    #define EXPECT_EQ(a, b) assert((a) == (b))
    #define EXPECT_NE(a, b) assert((a) != (b))
    #define EXPECT_TRUE(a) assert(a)
    #define EXPECT_FALSE(a) assert(!(a))
    #define TEST(suite, name) void suite##_##name()
    #define RUN_TEST(suite, name) do { \
        std::cout << "Running " #suite "." #name "..." << std::flush; \
        suite##_##name(); \
        std::cout << " PASSED" << std::endl; \
    } while(0)
#endif

using namespace middleware::data;
using namespace middleware::utils;

// ============================================================================
// VehicleState Tests
// ============================================================================

TEST(VehicleState, SizeCheck) {
    // Verify size is exactly 192 bytes as specified
    EXPECT_EQ(sizeof(VehicleState), 192);
}

TEST(VehicleState, DefaultConstruction) {
    VehicleState v;
    
    // All fields should be zero-initialized
    EXPECT_EQ(v.id, 0);
    EXPECT_EQ(v.timestamp, 0.0);
    EXPECT_EQ(v.x, 0.0);
    EXPECT_EQ(v.y, 0.0);
    EXPECT_EQ(v.speed, 0.0);
}

TEST(VehicleState, SetAndGet) {
    VehicleState v;
    
    v.id = 42;
    v.timestamp = 1.5;
    v.x = 100.0;
    v.y = 50.0;
    v.z = 0.0;
    v.heading = 3.14159;
    v.speed = 25.0;
    v.vx = 20.0;
    v.vy = 15.0;
    
    EXPECT_EQ(v.id, 42);
    EXPECT_EQ(v.timestamp, 1.5);
    EXPECT_EQ(v.x, 100.0);
    EXPECT_EQ(v.y, 50.0);
    EXPECT_EQ(v.heading, 3.14159);
    EXPECT_EQ(v.speed, 25.0);
}

TEST(VehicleState, Equality) {
    VehicleState v1, v2;
    
    v1.id = 1;
    v1.x = 10.0;
    v1.y = 20.0;
    
    v2.id = 1;
    v2.x = 10.0;
    v2.y = 20.0;
    
    EXPECT_TRUE(v1 == v2);
    
    v2.x = 11.0;
    EXPECT_TRUE(v1 != v2);
}

TEST(VehicleState, Serialization) {
    VehicleState original;
    original.id = 123;
    original.timestamp = 2.5;
    original.x = 100.0;
    original.y = 200.0;
    original.z = 1.5;
    original.heading = 1.57;
    original.speed = 30.0;
    
    // Serialize
    uint8_t buffer[1024];
    size_t size = serialize(original, buffer, sizeof(buffer));
    
    EXPECT_EQ(size, sizeof(VehicleState));
    
    // Deserialize
    VehicleState deserialized;
    bool success = deserialize(buffer, size, deserialized);
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(original == deserialized);
}

TEST(VehicleState, VectorSerialization) {
    std::vector<VehicleState> vehicles;
    
    // Create 3 vehicles
    for (int i = 0; i < 3; ++i) {
        VehicleState v;
        v.id = i;
        v.x = i * 10.0;
        v.y = i * 20.0;
        v.speed = 15.0 + i * 5.0;
        vehicles.push_back(v);
    }
    
    // Serialize
    uint8_t buffer[4096];
    size_t size = serializeVehicles(vehicles, buffer, sizeof(buffer));
    
    // Should be: 4 bytes (count) + 3 * 192 bytes (vehicles)
    EXPECT_EQ(size, sizeof(uint32_t) + 3 * sizeof(VehicleState));
    
    // Deserialize
    std::vector<VehicleState> deserialized;
    bool success = deserializeVehicles(buffer, size, deserialized);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(deserialized.size(), 3);
    
    for (size_t i = 0; i < vehicles.size(); ++i) {
        EXPECT_TRUE(vehicles[i] == deserialized[i]);
    }
}

// ============================================================================
// SceneState Tests
// ============================================================================

TEST(SceneState, DefaultConstruction) {
    SceneState s;
    
    EXPECT_EQ(s.timestamp, 0.0);
    EXPECT_EQ(s.temperature, 0.0);
    EXPECT_EQ(s.friction, 0.0);
    EXPECT_EQ(s.num_vehicles, 0);
}

TEST(SceneState, SetAndGet) {
    SceneState s;
    
    s.timestamp = 5.0;
    s.temperature = 20.0;
    s.visibility = 1000.0;
    s.friction = 0.8;
    s.speed_limit = 33.33; // ~120 km/h
    s.num_vehicles = 5;
    s.traffic_density = 2; // moderate
    
    EXPECT_EQ(s.temperature, 20.0);
    EXPECT_EQ(s.visibility, 1000.0);
    EXPECT_EQ(s.friction, 0.8);
    EXPECT_EQ(s.num_vehicles, 5);
}

TEST(SceneState, Serialization) {
    SceneState original;
    original.timestamp = 3.0;
    original.temperature = 25.0;
    original.friction = 0.9;
    original.num_vehicles = 10;
    
    // Serialize
    uint8_t buffer[1024];
    size_t size = serialize(original, buffer, sizeof(buffer));
    
    // Deserialize
    SceneState deserialized;
    bool success = deserialize(buffer, size, deserialized);
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(original == deserialized);
}

// ============================================================================
// ControlCommand Tests
// ============================================================================

TEST(ControlCommand, SizeCheck) {
    // Verify size is at most 512 bytes
    EXPECT_TRUE(sizeof(ControlCommand) <= 512);
}

TEST(ControlCommand, DefaultConstruction) {
    ControlCommand cmd;
    
    EXPECT_EQ(cmd.vehicleId, 0);
    EXPECT_EQ(cmd.mode, ControlCommand::Mode::NONE);
    EXPECT_EQ(cmd.speed.targetSpeed, 0.0);
}

TEST(ControlCommand, SpeedControl) {
    ControlCommand cmd;
    
    cmd.vehicleId = 42;
    cmd.timestamp = 1.0;
    cmd.mode = ControlCommand::Mode::SPEED;
    cmd.speed.targetSpeed = 20.0;
    cmd.speed.maxAcceleration = 2.0;
    cmd.speed.maxDeceleration = 3.0;
    
    EXPECT_EQ(cmd.vehicleId, 42);
    EXPECT_EQ(cmd.mode, ControlCommand::Mode::SPEED);
    EXPECT_EQ(cmd.speed.targetSpeed, 20.0);
    EXPECT_EQ(cmd.speed.maxAcceleration, 2.0);
}

TEST(ControlCommand, PositionControl) {
    ControlCommand cmd;
    
    cmd.vehicleId = 1;
    cmd.mode = ControlCommand::Mode::POSITION;
    cmd.position.x = 100.0;
    cmd.position.y = 50.0;
    cmd.position.z = 0.0;
    cmd.position.heading = 1.57;
    
    EXPECT_EQ(cmd.mode, ControlCommand::Mode::POSITION);
    EXPECT_EQ(cmd.position.x, 100.0);
    EXPECT_EQ(cmd.position.y, 50.0);
}

TEST(ControlCommand, TrajectoryControl) {
    ControlCommand cmd;
    
    cmd.vehicleId = 1;
    cmd.mode = ControlCommand::Mode::TRAJECTORY;
    cmd.trajectory.numPoints = 3;
    
    cmd.trajectory.waypoints[0].x = 0.0;
    cmd.trajectory.waypoints[0].y = 0.0;
    cmd.trajectory.waypoints[0].speed = 10.0;
    
    cmd.trajectory.waypoints[1].x = 50.0;
    cmd.trajectory.waypoints[1].y = 0.0;
    cmd.trajectory.waypoints[1].speed = 15.0;
    
    cmd.trajectory.waypoints[2].x = 100.0;
    cmd.trajectory.waypoints[2].y = 0.0;
    cmd.trajectory.waypoints[2].speed = 20.0;
    
    EXPECT_EQ(cmd.mode, ControlCommand::Mode::TRAJECTORY);
    EXPECT_EQ(cmd.trajectory.numPoints, 3);
    EXPECT_EQ(cmd.trajectory.waypoints[1].x, 50.0);
}

TEST(ControlCommand, Serialization) {
    ControlCommand original;
    original.vehicleId = 99;
    original.timestamp = 4.5;
    original.mode = ControlCommand::Mode::SPEED;
    original.speed.targetSpeed = 25.0;
    
    // Serialize
    uint8_t buffer[1024];
    size_t size = serialize(original, buffer, sizeof(buffer));
    
    // Deserialize
    ControlCommand deserialized;
    bool success = deserialize(buffer, size, deserialized);
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(original == deserialized);
}

// ============================================================================
// SensorData Tests
// ============================================================================

TEST(SensorData, DefaultConstruction) {
    SensorData s;
    
    EXPECT_EQ(s.vehicleId, 0);
    EXPECT_EQ(s.timestamp, 0.0);
    EXPECT_EQ(s.sensorType, SensorData::Type::CAMERA);
}

TEST(SensorData, GPSData) {
    SensorData s;
    
    s.vehicleId = 1;
    s.sensorType = SensorData::Type::GPS;
    s.gps.latitude = 37.7749;
    s.gps.longitude = -122.4194;
    s.gps.altitude = 10.0;
    s.gps.accuracy = 2.5;
    
    EXPECT_EQ(s.sensorType, SensorData::Type::GPS);
    EXPECT_EQ(s.gps.latitude, 37.7749);
    EXPECT_EQ(s.gps.longitude, -122.4194);
}

TEST(SensorData, Serialization) {
    SensorData original;
    original.vehicleId = 5;
    original.timestamp = 2.0;
    original.sensorType = SensorData::Type::LIDAR;
    original.lidar.numPoints = 1000;
    original.lidar.maxRange = 100.0;
    
    // Serialize
    uint8_t buffer[4096];
    size_t size = serialize(original, buffer, sizeof(buffer));
    
    // Deserialize
    SensorData deserialized;
    bool success = deserialize(buffer, size, deserialized);
    
    EXPECT_TRUE(success);
    EXPECT_TRUE(original == deserialized);
}

// ============================================================================
// Main Test Runner
// ============================================================================

#ifndef GTEST_FOUND
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Middleware Data Structures - Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // VehicleState tests
    RUN_TEST(VehicleState, SizeCheck);
    RUN_TEST(VehicleState, DefaultConstruction);
    RUN_TEST(VehicleState, SetAndGet);
    RUN_TEST(VehicleState, Equality);
    RUN_TEST(VehicleState, Serialization);
    RUN_TEST(VehicleState, VectorSerialization);
    
    // SceneState tests
    RUN_TEST(SceneState, DefaultConstruction);
    RUN_TEST(SceneState, SetAndGet);
    RUN_TEST(SceneState, Serialization);
    
    // ControlCommand tests
    RUN_TEST(ControlCommand, SizeCheck);
    RUN_TEST(ControlCommand, DefaultConstruction);
    RUN_TEST(ControlCommand, SpeedControl);
    RUN_TEST(ControlCommand, PositionControl);
    RUN_TEST(ControlCommand, TrajectoryControl);
    RUN_TEST(ControlCommand, Serialization);
    
    // SensorData tests
    RUN_TEST(SensorData, DefaultConstruction);
    RUN_TEST(SensorData, GPSData);
    RUN_TEST(SensorData, Serialization);
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "All tests PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
#endif
