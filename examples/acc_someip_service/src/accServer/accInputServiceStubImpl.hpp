/**
 * @file accInputServiceStubImpl.hpp
 * @brief Stub implementation for ACC Input Service
 * 
 * This class implements the service provider (server) for the ACC Input Service.
 * It receives data from the esmini middleware via SomeIP.
 */

#ifndef ACC_INPUT_SERVICE_STUB_IMPL_HPP
#define ACC_INPUT_SERVICE_STUB_IMPL_HPP

#include <CommonAPI/CommonAPI.hpp>
#include <v1/com/kpit/acc/services/accInputServiceInterfaceStubDefault.hpp>

using namespace v1::com::kpit::acc::services;

class accInputServiceStubImpl : public accInputServiceInterfaceStubDefault {
public:
    accInputServiceStubImpl();
    virtual ~accInputServiceStubImpl();
    
    /**
     * @brief Update all vehicle and target data from middleware
     * This is called by the middleware's SomeIP adapter
     */
    void updateFromMiddleware(
        double veh_speed,
        double veh_accel,
        double target_pos,
        double target_vel,
        double target_status
    );
    
    /**
     * @brief Update vehicle speed and derived parameters
     */
    void updateVehicleSpeed(double speed);
    
    /**
     * @brief Update vehicle acceleration
     */
    void updateVehicleAcceleration(double accel);
    
    /**
     * @brief Update target information
     */
    void updateTargetInfo(double position, double velocity, double speed, double status);
    
    /**
     * @brief Update path radius
     */
    void updatePathRadius(double radius);
    
    /**
     * @brief Set system power mode
     */
    void setSystemPowerMode(double mode);
    
    /**
     * @brief Set ACC ECU command
     */
    void setACCCommand(double command);

private:
    // Internal state
    double m_veh_speed;
    double m_veh_accel;
    double m_target_pos;
    double m_target_vel;
};

#endif // ACC_INPUT_SERVICE_STUB_IMPL_HPP
