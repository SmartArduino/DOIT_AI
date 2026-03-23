#ifndef EC801E_MQTT_H
#define EC801E_MQTT_H

#include "mqtt.h"

#include "at_uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string>
#include <functional>

#define EC801E_MQTT_CONNECT_TIMEOUT_MS 10000

#define EC801E_MQTT_INITIALIZED_EVENT BIT0
#define EC801E_MQTT_CONNECTED_EVENT BIT1
#define EC801E_MQTT_DISCONNECTED_EVENT BIT2
#define EC801E_MQTT_PUBLISH_COMPLETE BIT3
#define EC801E_MQTT_PUBLISH_FAILED BIT4
#define EC801E_MQTT_OPEN_COMPLETE BIT5
#define EC801E_MQTT_OPEN_FAILED BIT6

class Ec801EMqtt : public Mqtt {
public:
    Ec801EMqtt(std::shared_ptr<AtUart> at_uart, int mqtt_id);
    ~Ec801EMqtt();

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
    int error_code_ = 0;
    EventGroupHandle_t event_group_handle_;
    std::string message_payload_;

    std::list<UrcCallback>::iterator urc_callback_it_;

    std::string ErrorToString(int error_code);
};

#endif