#ifndef ESP_UDP_H
#define ESP_UDP_H

#include "udp.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#define ESP_UDP_EVENT_RECEIVE_TASK_EXIT 1

class EspUdp : public Udp {
public:
    EspUdp();
    ~EspUdp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    int udp_fd_;
    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;

    void ReceiveTask();
};

#endif // ESP_UDP_H
