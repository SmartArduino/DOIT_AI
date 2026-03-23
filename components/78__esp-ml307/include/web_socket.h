#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <functional>
#include <string>
#include <map>
#include <thread>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "tcp.h"

class NetworkInterface;

class WebSocket {
public:
    WebSocket(NetworkInterface* network, int connect_id);
    ~WebSocket();

    void SetHeader(const char* key, const char* value);
    void SetReceiveBufferSize(size_t size);
    bool IsConnected() const;
    bool Connect(const char* uri);
    bool Send(const std::string& data);
    bool Send(const void* data, size_t len, bool binary = false, bool fin = true);
    void Ping();
    void Close();

    void OnConnected(std::function<void()> callback);
    void OnDisconnected(std::function<void()> callback);
    void OnData(std::function<void(const char*, size_t, bool binary)> callback);
    void OnError(std::function<void(int)> callback);

private:
    NetworkInterface* network_;
    int connect_id_;
    std::unique_ptr<Tcp> tcp_;
    bool continuation_ = false;
    size_t receive_buffer_size_ = 2048;
    std::string receive_buffer_;
    bool handshake_completed_ = false;
    bool connected_ = false;
    
    // FreeRTOS 事件组用于同步握手
    EventGroupHandle_t handshake_event_group_;
    static const EventBits_t HANDSHAKE_SUCCESS_BIT = BIT0;
    static const EventBits_t HANDSHAKE_FAILED_BIT = BIT1;

    std::map<std::string, std::string> headers_;
    std::function<void(const char*, size_t, bool binary)> on_data_;
    std::function<void(int)> on_error_;
    std::function<void()> on_connected_;
    std::function<void()> on_disconnected_;

    void OnTcpData(const std::string& data);
    bool SendControlFrame(uint8_t opcode, const void* data, size_t len);
};

#endif // WEBSOCKET_H
