#pragma once

#include "middleware/data/vehicle_state.h"
#include "middleware/data/scene_state.h"
#include "middleware/data/control_command.h"
#include <vector>
#include <cstdint>
#include <functional>

namespace middleware::ipc {

// Callback type for asynchronous command reception
// Called automatically when a control command is received
using CommandCallback = std::function<void(const data::ControlCommand& cmd)>;

// Statistics for IPC monitoring
struct IPCStatistics {
    uint64_t messagesSent = 0;
    uint64_t messagesReceived = 0;
    uint64_t bytesSent = 0;
    uint64_t bytesReceived = 0;
    uint64_t sendErrors = 0;
    uint64_t receiveErrors = 0;
    double lastPublishTime = 0.0;
    double lastReceiveTime = 0.0;
};

// Abstract IPC adapter interface
// All communication adapters (UDP, shared memory, DDS, etc.) implement this
class IIPCAdapter {
public:
    virtual ~IIPCAdapter() = default;
    
    // Initialize the IPC adapter (create sockets, allocate memory, etc.)
    virtual bool initialize() = 0;
    
    // Terminate the IPC adapter (close sockets, free resources, etc.)
    virtual bool terminate() = 0;
    
    // Publish vehicle states to applications (perception data)
    // Returns true if successful
    virtual bool publishVehicles(const std::vector<data::VehicleState>& vehicles) = 0;
    
    // Publish scene state to applications (environment data)
    // Returns true if successful
    virtual bool publishScene(const data::SceneState& scene) = 0;
    
    // ========== Event-Driven Asynchronous Command Reception ==========
    
    // Register callback for asynchronous command reception
    // The callback will be invoked automatically when commands arrive (event-driven)
    // Runs in background thread - callback must be thread-safe
    // Returns true if callback registered successfully
    virtual bool registerCommandCallback(CommandCallback callback) = 0;
    
    // Unregister the command callback (stops async reception)
    virtual void unregisterCommandCallback() = 0;
    
    // Start asynchronous listener thread (for callback-based reception)
    // Must be called after registerCommandCallback() to begin receiving
    virtual bool startAsyncListener() = 0;
    
    // Stop asynchronous listener thread
    virtual void stopAsyncListener() = 0;
    
    // Check if async listener is running
    virtual bool isAsyncListenerRunning() const = 0;
    
    // Get IPC statistics
    virtual IPCStatistics getStatistics() const = 0;
    
    // Check if adapter is ready
    virtual bool isReady() const = 0;
};

} // namespace middleware::ipc
