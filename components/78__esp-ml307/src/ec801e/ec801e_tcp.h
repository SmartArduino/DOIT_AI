#ifndef EC801E_TCP_H
#define EC801E_TCP_H

#include "tcp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <string>

#define EC801E_TCP_CONNECTED BIT0
#define EC801E_TCP_DISCONNECTED BIT1
#define EC801E_TCP_ERROR BIT2
#define EC801E_TCP_SEND_COMPLETE BIT3
#define EC801E_TCP_SEND_FAILED BIT4
#define EC801E_TCP_INITIALIZED BIT5

#define TCP_CONNECT_TIMEOUT_MS 10000

class Ec801ETcp : public Tcp {
public:
    Ec801ETcp(std::shared_ptr<AtUart> at_uart, int tcp_id);
    ~Ec801ETcp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    std::shared_ptr<AtUart> at_uart_;
    int tcp_id_;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
};

#endif // EC801E_TCP_H
