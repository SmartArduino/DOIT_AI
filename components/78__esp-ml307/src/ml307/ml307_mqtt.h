#ifndef ML307_MQTT_H
#define ML307_MQTT_H

#include "mqtt.h"

#include "at_uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string>
#include <functional>

#define MQTT_CONNECT_TIMEOUT_MS 10000

#define MQTT_INITIALIZED_EVENT BIT0
#define MQTT_CONNECTED_EVENT BIT1
#define MQTT_DISCONNECTED_EVENT BIT2

class Ml307Mqtt : public Mqtt {
public:
    Ml307Mqtt(std::shared_ptr<AtUart> at_uart, int mqtt_id);
    ~Ml307Mqtt();

    bool Connect(const std::string broker_address, int broker_port, const std::string client_id, const std::string username, const std::string password);
    void Disconnect();
    bool Publish(const std::string topic, const std::string payload, int qos = 0);
    bool Subscribe(const std::string topic, int qos = 0);
    bool Unsubscribe(const std::string topic);
    bool IsConnected();

private:
    std::shared_ptr<AtUart> at_uart_;
    int mqtt_id_;
    bool connected_ = false;
    EventGroupHandle_t event_group_handle_;
    std::string message_payload_;

    std::list<UrcCallback>::iterator urc_callback_it_;

    std::string ErrorToString(int error_code);
};

#endif