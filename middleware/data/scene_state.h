#pragma once

#include <cstdint>
#include <cstring>

namespace middleware::data {

// Scene state - environmental and road context
struct SceneState {
    // ========== Time (8 bytes) ==========
    double timestamp;               // Simulation time [s]
    
    // ========== Weather Conditions (32 bytes) ==========
    double temperature;             // Air temperature [°C]
    double visibility;              // Visibility distance [m]
    double precipitation;           // Precipitation intensity [mm/h]
    double wind_speed;              // Wind speed [m/s]
    
    // ========== Road Conditions (32 bytes) ==========
    double friction;                // Road friction coefficient [0-1]
    double curvature;               // Road curvature [1/m]
    double grade;                   // Road grade/slope [%]
    double speed_limit;             // Speed limit [m/s]
    
    // ========== Lighting (16 bytes) ==========
    uint32_t time_of_day;           // Hour [0-23]
    double ambient_light;           // Ambient light level [lux]
    uint32_t weather_type;          // 0=clear, 1=cloudy, 2=rain, 3=fog, 4=snow
    
    // ========== Traffic Context (24 bytes) ==========
    uint32_t traffic_density;       // 0=free, 1=light, 2=moderate, 3=heavy, 4=congested
    uint32_t num_vehicles;          // Total number of vehicles in scene
    uint32_t num_pedestrians;       // Total number of pedestrians in scene
    uint32_t num_obstacles;         // Total number of static obstacles
    double ego_distance_to_junction;// Distance to next junction [m]
    
    // ========== Reserved (32 bytes) ==========
    uint32_t reserved[8];           // Future use
    
    // Default constructor - zero initialize
    SceneState() {
        std::memset(this, 0, sizeof(SceneState));
    }
    
    // Equality operator for testing
    bool operator==(const SceneState& other) const {
        return std::memcmp(this, &other, sizeof(SceneState)) == 0;
    }
    
    bool operator!=(const SceneState& other) const {
        return !(*this == other);
    }
};

} // namespace middleware::data
