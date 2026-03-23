#ifndef EC801E_UDP_H
#define EC801E_UDP_H

#include "udp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define EC801E_UDP_CONNECTED BIT0
#define EC801E_UDP_DISCONNECTED BIT1
#define EC801E_UDP_ERROR BIT2
#define EC801E_UDP_SEND_COMPLETE BIT3
#define EC801E_UDP_SEND_FAILED BIT4
#define EC801E_UDP_INITIALIZED BIT5

#define UDP_CONNECT_TIMEOUT_MS 10000

class Ec801EUdp : public Udp {
public:
    Ec801EUdp(std::shared_ptr<AtUart> at_uart, int udp_id);
    ~Ec801EUdp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    std::shared_ptr<AtUart> at_uart_;
    int udp_id_;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
};

#endif // EC801E_UDP_H
