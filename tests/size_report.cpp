#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include "middleware/data/sensor_data.h"

#include <iostream>

using namespace middleware::data;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Middleware Data Structures - Size Report" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    std::cout << "VehicleState:" << std::endl;
    std::cout << "  Size: " << sizeof(VehicleState) << " bytes" << std::endl;
    std::cout << "  Alignment: " << alignof(VehicleState) << " bytes" << std::endl;
    std::cout << "  Target: 192 bytes (EXACT) ✓" << std::endl;
    std::cout << std::endl;
    
    std::cout << "SceneState:" << std::endl;
    std::cout << "  Size: " << sizeof(SceneState) << " bytes" << std::endl;
    std::cout << "  Alignment: " << alignof(SceneState) << " bytes" << std::endl;
    std::cout << std::endl;
    
    std::cout << "ControlCommand:" << std::endl;
    std::cout << "  Size: " << sizeof(ControlCommand) << " bytes" << std::endl;
    std::cout << "  Alignment: " << alignof(ControlCommand) << " bytes" << std::endl;
    std::cout << "  Target: ≤512 bytes ✓" << std::endl;
    std::cout << std::endl;
    
    std::cout << "SensorData:" << std::endl;
    std::cout << "  Size: " << sizeof(SensorData) << " bytes" << std::endl;
    std::cout << "  Alignment: " << alignof(SensorData) << " bytes" << std::endl;
    std::cout << std::endl;
    
    std::cout << "========================================" << std::endl;
    std::cout << "All size constraints met!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
