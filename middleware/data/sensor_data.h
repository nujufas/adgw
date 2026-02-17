#pragma once

#include <cstdint>
#include <cstring>

namespace middleware::data {

// Sensor data - raw perception (optional, for advanced applications)
struct SensorData {
    // ========== Identity (16 bytes) ==========
    uint64_t vehicleId;             // Vehicle this sensor belongs to
    double timestamp;               // Sensor data timestamp [s]
    
    // ========== Sensor Type (8 bytes) ==========
    enum class Type : uint32_t {
        CAMERA = 0,                 // Camera/vision sensor
        LIDAR = 1,                  // LiDAR point cloud
        RADAR = 2,                  // Radar detections
        ULTRASONIC = 3,             // Ultrasonic sensors
        GPS = 4,                    // GPS/GNSS
        IMU = 5,                    // Inertial measurement unit
        GROUND_TRUTH = 6            // Perfect sensor (simulation)
    };
    Type sensorType;
    uint32_t sensorId;              // Sensor ID on vehicle
    
    // ========== Detection Data (64 bytes) ==========
    struct {
        uint32_t numDetections;     // Number of detected objects
        uint32_t maxDetections;     // Maximum capacity (16)
        struct Detection {
            uint64_t objectId;      // Detected object ID
            double range;           // Distance to object [m]
            double azimuth;         // Horizontal angle [rad]
            double elevation;       // Vertical angle [rad]
            double rangeRate;       // Relative velocity [m/s]
            uint32_t objectClass;   // 0=vehicle, 1=pedestrian, 2=cyclist, 3=obstacle
            float confidence;       // Detection confidence [0-1]
        } detections[16];           // Fixed-size detection array (simplified)
    } radar;
    
    // ========== Point Cloud Data (32 bytes) ==========
    struct {
        uint32_t numPoints;         // Number of points (reference to full data)
        uint32_t dataOffset;        // Offset to full point cloud (if stored externally)
        double minRange;            // Minimum range [m]
        double maxRange;            // Maximum range [m]
        double horizontalFOV;       // Horizontal field of view [rad]
        double verticalFOV;         // Vertical field of view [rad]
    } lidar;
    
    // ========== Image Data (32 bytes) ==========
    struct {
        uint32_t width;             // Image width [pixels]
        uint32_t height;            // Image height [pixels]
        uint32_t format;            // 0=RGB, 1=RGBA, 2=BGR, 3=BGRA, 4=Grayscale
        uint32_t dataOffset;        // Offset to full image (if stored externally)
        double horizontalFOV;       // Horizontal field of view [rad]
        double verticalFOV;         // Vertical field of view [rad]
    } camera;
    
    // ========== GPS Data (48 bytes) ==========
    struct {
        double latitude;            // Latitude [degrees]
        double longitude;           // Longitude [degrees]
        double altitude;            // Altitude [m]
        double accuracy;            // Position accuracy [m]
        double heading;             // GPS heading [rad]
        double speed;               // GPS speed [m/s]
    } gps;
    
    // ========== IMU Data (48 bytes) ==========
    struct {
        double accel_x;             // Linear acceleration X [m/s²]
        double accel_y;             // Linear acceleration Y [m/s²]
        double accel_z;             // Linear acceleration Z [m/s²]
        double gyro_x;              // Angular velocity X [rad/s]
        double gyro_y;              // Angular velocity Y [rad/s]
        double gyro_z;              // Angular velocity Z [rad/s]
    } imu;
    
    // ========== Reserved (16 bytes) ==========
    uint32_t reserved[4];           // Future use
    
    // Default constructor - zero initialize
    SensorData() {
        std::memset(this, 0, sizeof(SensorData));
    }
    
    // Equality operator for testing
    bool operator==(const SensorData& other) const {
        return std::memcmp(this, &other, sizeof(SensorData)) == 0;
    }
    
    bool operator!=(const SensorData& other) const {
        return !(*this == other);
    }
};

} // namespace middleware::data
