#include "middleware/ipc/udp_adapter.h"
#include "middleware/utils/serialization.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

namespace middleware::ipc {

UDPAdapter::UDPAdapter()
    : config_{}
    , publishSocket_(-1)
    , receiveSocket_(-1)
    , publishAddr_{}
    , receiveAddr_{}
    , initialized_(false)
    , commandCallback_(nullptr)
    , listenerRunning_(false)
    , stats_{}
{
}

UDPAdapter::UDPAdapter(const UDPConfig& config)
    : config_(config)
    , publishSocket_(-1)
    , receiveSocket_(-1)
    , publishAddr_{}
    , receiveAddr_{}
    , initialized_(false)
    , commandCallback_(nullptr)
    , listenerRunning_(false)
    , stats_{}
{
}

UDPAdapter::~UDPAdapter() {
    stopAsyncListener();
    terminate();
}

bool UDPAdapter::initialize() {
    if (initialized_) {
        return true;
    }
    
    // Create sockets
    if (!createPublishSocket()) {
        return false;
    }
    
    if (!createReceiveSocket()) {
        closePublishSocket();
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool UDPAdapter::terminate() {
    if (!initialized_) {
        return true;
    }
    
    closePublishSocket();
    closeReceiveSocket();
    
    initialized_ = false;
    return true;
}

bool UDPAdapter::createPublishSocket() {
    // Create UDP socket
    publishSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (publishSocket_ < 0) {
        return false;
    }
    
    // Set socket options
    int sendbuf = config_.sendBufferSize;
    if (setsockopt(publishSocket_, SOL_SOCKET, SO_SNDBUF, 
                   &sendbuf, sizeof(sendbuf)) < 0) {
        close(publishSocket_);
        publishSocket_ = -1;
        return false;
    }
    
    // Enable multicast if configured
    if (config_.enableMulticast) {
        // Set multicast TTL
        unsigned char ttl = static_cast<unsigned char>(config_.multicastTTL);
        if (setsockopt(publishSocket_, IPPROTO_IP, IP_MULTICAST_TTL,
                      &ttl, sizeof(ttl)) < 0) {
            close(publishSocket_);
            publishSocket_ = -1;
            return false;
        }
        
        // Enable multicast loopback (for testing on same machine)
        unsigned char loop = 1;
        if (setsockopt(publishSocket_, IPPROTO_IP, IP_MULTICAST_LOOP,
                      &loop, sizeof(loop)) < 0) {
            close(publishSocket_);
            publishSocket_ = -1;
            return false;
        }
    }
    
    // Setup publish address
    std::memset(&publishAddr_, 0, sizeof(publishAddr_));
    publishAddr_.sin_family = AF_INET;
    publishAddr_.sin_port = htons(config_.publishPort);
    if (inet_pton(AF_INET, config_.publishAddress.c_str(), 
                  &publishAddr_.sin_addr) <= 0) {
        close(publishSocket_);
        publishSocket_ = -1;
        return false;
    }
    
    return true;
}

bool UDPAdapter::createReceiveSocket() {
    // Create UDP socket
    receiveSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiveSocket_ < 0) {
        return false;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(receiveSocket_, F_GETFL, 0);
    if (flags < 0 || fcntl(receiveSocket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
        return false;
    }
    
    // Allow address reuse
    int reuse = 1;
    if (setsockopt(receiveSocket_, SOL_SOCKET, SO_REUSEADDR,
                   &reuse, sizeof(reuse)) < 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
        return false;
    }
    
    // Set receive buffer size
    int recvbuf = config_.receiveBufferSize;
    if (setsockopt(receiveSocket_, SOL_SOCKET, SO_RCVBUF,
                   &recvbuf, sizeof(recvbuf)) < 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
        return false;
    }
    
    // Bind to receive address
    std::memset(&receiveAddr_, 0, sizeof(receiveAddr_));
    receiveAddr_.sin_family = AF_INET;
    receiveAddr_.sin_port = htons(config_.receivePort);
    if (inet_pton(AF_INET, config_.receiveAddress.c_str(),
                  &receiveAddr_.sin_addr) <= 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
        return false;
    }
    
    if (bind(receiveSocket_, (struct sockaddr*)&receiveAddr_,
             sizeof(receiveAddr_)) < 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
        return false;
    }
    
    return true;
}

void UDPAdapter::closePublishSocket() {
    if (publishSocket_ >= 0) {
        close(publishSocket_);
        publishSocket_ = -1;
    }
}

void UDPAdapter::closeReceiveSocket() {
    if (receiveSocket_ >= 0) {
        close(receiveSocket_);
        receiveSocket_ = -1;
    }
}

bool UDPAdapter::publishVehicles(const std::vector<data::VehicleState>& vehicles) {
    if (!initialized_ || publishSocket_ < 0) {
        return false;
    }
    
    // Serialize vehicles to binary buffer
    uint8_t buffer[65536];  // 64 KB buffer
    size_t offset = 0;
    
    // Serialize vehicle count (4 bytes)
    uint32_t count = static_cast<uint32_t>(vehicles.size());
    if (offset + sizeof(count) > sizeof(buffer)) {
        stats_.sendErrors++;
        return false;
    }
    std::memcpy(buffer + offset, &count, sizeof(count));
    offset += sizeof(count);
    
    // Serialize each vehicle
    for (const auto& vehicle : vehicles) {
        size_t remaining = sizeof(buffer) - offset;
        size_t serialized = utils::serialize(vehicle, buffer + offset, remaining);
        if (serialized == 0) {
            stats_.sendErrors++;
            return false;
        }
        offset += serialized;
    }
    
    // Send via UDP
    ssize_t sent = sendto(publishSocket_, buffer, offset, 0,
                         (struct sockaddr*)&publishAddr_, sizeof(publishAddr_));
    
    if (sent > 0) {
        stats_.messagesSent++;
        stats_.bytesSent += sent;
        return true;
    } else {
        stats_.sendErrors++;
        return false;
    }
}

bool UDPAdapter::publishScene(const data::SceneState& scene) {
    if (!initialized_ || publishSocket_ < 0) {
        return false;
    }
    
    // Serialize scene to binary buffer
    uint8_t buffer[1024];
    size_t serialized = utils::serialize(scene, buffer, sizeof(buffer));
    
    if (serialized == 0) {
        stats_.sendErrors++;
        return false;
    }
    
    // Send via UDP
    ssize_t sent = sendto(publishSocket_, buffer, serialized, 0,
                         (struct sockaddr*)&publishAddr_, sizeof(publishAddr_));
    
    if (sent > 0) {
        stats_.messagesSent++;
        stats_.bytesSent += sent;
        return true;
    } else {
        stats_.sendErrors++;
        return false;
    }
}

bool UDPAdapter::receiveCommandInternal(data::ControlCommand& cmd) {
    if (!initialized_ || receiveSocket_ < 0) {
        return false;
    }
    
    // Receive buffer
    uint8_t buffer[1024];
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);
    
    // Non-blocking receive
    ssize_t received = recvfrom(receiveSocket_, buffer, sizeof(buffer),
                               0, (struct sockaddr*)&from, &fromLen);
    
    if (received > 0) {
        // Deserialize command
        if (utils::deserialize(buffer, received, cmd)) {
            stats_.messagesReceived++;
            stats_.bytesReceived += received;
            return true;
        } else {
            stats_.receiveErrors++;
            return false;
        }
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        // Real error (not just no data available)
        stats_.receiveErrors++;
    }
    
    return false;
}

bool UDPAdapter::registerCommandCallback(CommandCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    commandCallback_ = callback;
    return callback != nullptr;
}

void UDPAdapter::unregisterCommandCallback() {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    commandCallback_ = nullptr;
}

bool UDPAdapter::startAsyncListener() {
    if (!initialized_ || receiveSocket_ < 0) {
        return false;
    }
    
    if (listenerRunning_.load()) {
        return true; // Already running
    }
    
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (!commandCallback_) {
            return false; // No callback registered
        }
    }
    
    listenerRunning_.store(true);
    listenerThread_ = std::thread(&UDPAdapter::asyncListenerThread, this);
    
    return true;
}

void UDPAdapter::stopAsyncListener() {
    if (!listenerRunning_.load()) {
        return; // Already stopped
    }
    
    listenerRunning_.store(false);
    
    if (listenerThread_.joinable()) {
        listenerThread_.join();
    }
}

bool UDPAdapter::isAsyncListenerRunning() const {
    return listenerRunning_.load();
}

void UDPAdapter::asyncListenerThread() {
    while (listenerRunning_.load()) {
        data::ControlCommand cmd;
        
        // Try to receive a command
        if (receiveCommandInternal(cmd)) {
            // Invoke callback if registered
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (commandCallback_) {
                commandCallback_(cmd);
            }
        } else {
            // No data available, sleep briefly to avoid busy-wait
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

IPCStatistics UDPAdapter::getStatistics() const {
    return stats_;
}

bool UDPAdapter::isReady() const {
    return initialized_ && publishSocket_ >= 0 && receiveSocket_ >= 0;
}

} // namespace middleware::ipc
