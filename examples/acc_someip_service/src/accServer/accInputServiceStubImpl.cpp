/**
 * @file accInputServiceStubImpl.cpp
 * @brief Stub implementation for ACC Input Service
 */

#include "accInputServiceStubImpl.hpp"
#include <iostream>
#include <cmath>

accInputServiceStubImpl::accInputServiceStubImpl()
    : m_veh_speed(0.0),
      m_veh_accel(0.0),
      m_target_pos(0.0),
      m_target_vel(0.0)
{
    std::cout << "ACC Input Service Stub created" << std::endl;
    
    // Initialize all attributes with default values
    setACC_ECU_CMDAttribute(0.0);
    setEnSpd_radpsAttribute(0.0);
    setSysPwrMdAttribute(1.0);  // Power mode ON
    setTrgt_Px_mAttribute(0.0);
    setTrgt_Vx_mpsAttribute(0.0);
    setTrgt_spd_mpsAttribute(0.0);
    setTrgt_statusAttribute(0.0);
    setVeh_PathRadius_mAttribute(0.0);
    setVeh_accel_mps2Attribute(0.0);
    setVeh_spd_mpsAttribute(0.0);
}

accInputServiceStubImpl::~accInputServiceStubImpl() {
    std::cout << "ACC Input Service Stub destroyed" << std::endl;
}

void accInputServiceStubImpl::updateFromMiddleware(
    double veh_speed,
    double veh_accel,
    double target_pos,
    double target_vel,
    double target_status)
{
    updateVehicleSpeed(veh_speed);
    updateVehicleAcceleration(veh_accel);
    updateTargetInfo(target_pos, target_vel, target_vel, target_status);
}

void accInputServiceStubImpl::updateVehicleSpeed(double speed) {
    m_veh_speed = speed;
    setVeh_spd_mpsAttribute(speed);
    
    // Also update engine speed (simplified calculation)
    // Assuming: engine_rpm = speed_mps * gear_ratio * wheel_circumference_factor
    // Simplified: engine_rpm = speed * 2.5 (approximation for typical gear)
    double engine_rpm = speed * 2.5;
    double engine_radps = (engine_rpm * 2 * M_PI) / 60.0;
    setEnSpd_radpsAttribute(engine_radps);
    
    std::cout << "Updated vehicle speed: " << speed << " m/s, Engine: " 
              << engine_radps << " rad/s" << std::endl;
}

void accInputServiceStubImpl::updateVehicleAcceleration(double accel) {
    m_veh_accel = accel;
    setVeh_accel_mps2Attribute(accel);
    std::cout << "Updated vehicle acceleration: " << accel << " m/s^2" << std::endl;
}

void accInputServiceStubImpl::updateTargetInfo(double position, double velocity, 
                                                double speed, double status) {
    m_target_pos = position;
    m_target_vel = velocity;
    
    setTrgt_Px_mAttribute(position);
    setTrgt_Vx_mpsAttribute(velocity);
    setTrgt_spd_mpsAttribute(speed);
    setTrgt_statusAttribute(status);
    
    std::cout << "Updated target info - Pos: " << position 
              << " m, Vel: " << velocity << " m/s, Status: " << status << std::endl;
}

void accInputServiceStubImpl::updatePathRadius(double radius) {
    setVeh_PathRadius_mAttribute(radius);
    std::cout << "Updated path radius: " << radius << " m" << std::endl;
}

void accInputServiceStubImpl::setSystemPowerMode(double mode) {
    setSysPwrMdAttribute(mode);
    std::cout << "Set system power mode: " << mode << std::endl;
}

void accInputServiceStubImpl::setACCCommand(double command) {
    setACC_ECU_CMDAttribute(command);
    std::cout << "Set ACC command: " << command << std::endl;
}
