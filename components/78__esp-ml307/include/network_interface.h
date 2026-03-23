#ifndef NETWORK_INTERFACE_H
#define NETWORK_INTERFACE_H

#include <memory>
#include "http.h"
#include "tcp.h"
#include "udp.h"
#include "mqtt.h"
#include "web_socket.h"

class NetworkInterface {
public:
    virtual ~NetworkInterface() = default;

    // 连接创建接口（纯虚函数，由子类实现）
    virtual std::unique_ptr<Http> CreateHttp(int connect_id = -1) = 0;
    virtual std::unique_ptr<Tcp> CreateTcp(int connect_id = -1) = 0;
    virtual std::unique_ptr<Tcp> CreateSsl(int connect_id = -1) = 0;
    virtual std::unique_ptr<Udp> CreateUdp(int connect_id = -1) = 0;
    virtual std::unique_ptr<Mqtt> CreateMqtt(int connect_id = -1) = 0;
    virtual std::unique_ptr<WebSocket> CreateWebSocket(int connect_id = -1) = 0;
};

#endif // NETWORK_INTERFACE_H
