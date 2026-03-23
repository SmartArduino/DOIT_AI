#ifndef ML307_TCP_H
#define ML307_TCP_H

#include "tcp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <string>

#define ML307_TCP_CONNECTED BIT0
#define ML307_TCP_DISCONNECTED BIT1
#define ML307_TCP_ERROR BIT2
#define ML307_TCP_SEND_COMPLETE BIT4
#define ML307_TCP_INITIALIZED BIT5

#define TCP_CONNECT_TIMEOUT_MS 10000

class Ml307Tcp : public Tcp {
public:
    Ml307Tcp(std::shared_ptr<AtUart> at_uart, int tcp_id);
    virtual ~Ml307Tcp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

protected:
    std::shared_ptr<AtUart> at_uart_;
    int tcp_id_;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
    
    // 虚函数允许子类自定义SSL配置
    virtual bool ConfigureSsl(int port);
};

#endif // ML307_TCP_H 