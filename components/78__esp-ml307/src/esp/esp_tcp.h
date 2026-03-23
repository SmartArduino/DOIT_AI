#ifndef _ESP_TCP_H_
#define _ESP_TCP_H_

#include "tcp.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#define ESP_TCP_EVENT_RECEIVE_TASK_EXIT 1

class EspTcp : public Tcp {
public:
    EspTcp();
    ~EspTcp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    int tcp_fd_ = -1;
    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;

    void ReceiveTask();
};

#endif // _ESP_TCP_H_