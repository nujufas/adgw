#pragma once

#include "middleware/ipc/ipc_interface.h"
#include <string>
#include <netinet/in.h>
#include <thread>
#include <atomic>
#include <mutex>

namespace middleware::ipc {

// Configuration for UDP adapter
struct UDPConfig {
    // Publish settings (middleware -> applications)
    std::string publishAddress = "239.255.0.1";  // Multicast group
    int publishPort = 5000;
    
    // Receive settings (applications -> middleware)
    std::string receiveAddress = "0.0.0.0";      // Listen on all interfaces
    int receivePort = 5001;
    
    // Socket options
    int sendBufferSize = 262144;     // 256 KB send buffer
    int receiveBufferSize = 262144;  // 256 KB receive buffer
    bool enableMulticast = true;
    int multicastTTL = 1;            // Local network only
};

// UDP-based IPC adapter
// Uses UDP sockets for publish/subscribe communication
// Supports multicast for efficient one-to-many distribution
class UDPAdapter : public IIPCAdapter {
public:
    UDPAdapter();
    explicit UDPAdapter(const UDPConfig& config);
    ~UDPAdapter() override;
    
    // IIPCAdapter interface
    bool initialize() override;
    bool terminate() override;
    bool publishVehicles(const std::vector<data::VehicleState>& vehicles) override;
    bool publishScene(const data::SceneState& scene) override;
    bool registerCommandCallback(CommandCallback callback) override;
    void unregisterCommandCallback() override;
    bool startAsyncListener() override;
    void stopAsyncListener() override;
    bool isAsyncListenerRunning() const override;
    IPCStatistics getStatistics() const override;
    bool isReady() const override;
    
    // UDP-specific methods
    const UDPConfig& getConfig() const { return config_; }
    
private:
    // Socket creation and configuration
    bool createPublishSocket();
    bool createReceiveSocket();
    void closePublishSocket();
    void closeReceiveSocket();
    
    // Internal command reception (used by async thread)
    bool receiveCommandInternal(data::ControlCommand& cmd);
    
    // Async listener thread function
    void asyncListenerThread();
    
    // Configuration
    UDPConfig config_;
    
    // Socket file descriptors
    int publishSocket_;
    int receiveSocket_;
    
    // Socket addresses
    struct sockaddr_in publishAddr_;
    struct sockaddr_in receiveAddr_;
    
    // State
    bool initialized_;
    
    // Async callback support
    CommandCallback commandCallback_;
    std::thread listenerThread_;
    std::atomic<bool> listenerRunning_;
    std::mutex callbackMutex_;
    
    // Statistics
    IPCStatistics stats_;
};

} // namespace middleware::ipc
