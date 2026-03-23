#ifndef ESP_NETWORK_H
#define ESP_NETWORK_H

#include "network_interface.h"

class EspNetwork : public NetworkInterface {
public:
    EspNetwork();
    ~EspNetwork();

    std::unique_ptr<Http> CreateHttp(int connect_id = -1) override;
    std::unique_ptr<Tcp> CreateTcp(int connect_id = -1) override;
    std::unique_ptr<Tcp> CreateSsl(int connect_id = -1) override;
    std::unique_ptr<Udp> CreateUdp(int connect_id = -1) override;
    std::unique_ptr<Mqtt> CreateMqtt(int connect_id = -1) override;
    std::unique_ptr<WebSocket> CreateWebSocket(int connect_id = -1) override;
};

#endif // ESP_NETWORK_H
