#ifndef EC801E_SSL_H
#define EC801E_SSL_H

#include "tcp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define EC801E_SSL_CONNECTED BIT0
#define EC801E_SSL_DISCONNECTED BIT1
#define EC801E_SSL_ERROR BIT2
#define EC801E_SSL_SEND_COMPLETE BIT3
#define EC801E_SSL_SEND_FAILED BIT4
#define EC801E_SSL_INITIALIZED BIT5

#define SSL_CONNECT_TIMEOUT_MS 10000

class Ec801ESsl : public Tcp {
public:
    Ec801ESsl(std::shared_ptr<AtUart> at_uart, int ssl_id);
    ~Ec801ESsl();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    std::shared_ptr<AtUart> at_uart_;
    int ssl_id_;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
};

#endif // EC801E_SSL_H
