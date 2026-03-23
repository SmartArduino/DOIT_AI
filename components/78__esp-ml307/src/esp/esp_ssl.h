#ifndef _ESP_SSL_H_
#define _ESP_SSL_H_

#include "tcp.h"
#include <esp_tls.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#define ESP_SSL_EVENT_RECEIVE_TASK_EXIT 1

class EspSsl : public Tcp {
public:
    EspSsl();
    ~EspSsl();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;

private:
    esp_tls_t* tls_client_ = nullptr;
    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;

    void ReceiveTask();
};

#endif // _ESP_SSL_H_