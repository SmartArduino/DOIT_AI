#include "ml307_mqtt.h"
#include <esp_log.h>

static const char *TAG = "Ml307Mqtt";

Ml307Mqtt::Ml307Mqtt(std::shared_ptr<AtUart> at_uart, int mqtt_id) : at_uart_(at_uart), mqtt_id_(mqtt_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "MQTTURC" && arguments.size() >= 2) {
            if (arguments[1].int_value == mqtt_id_) {
                auto type = arguments[0].string_value;
                if (type == "conn") {
                    if (arguments[2].int_value == 0) {
                        connected_ = true;
                        xEventGroupSetBits(event_group_handle_, MQTT_CONNECTED_EVENT);
                    } else {
                        if (connected_) {
                            connected_ = false;
                            if (on_disconnected_callback_) {
                                on_disconnected_callback_();
                            }
                        }
                        xEventGroupSetBits(event_group_handle_, MQTT_DISCONNECTED_EVENT);
                    }
                    ESP_LOGI(TAG, "MQTT connection state: %s", ErrorToString(arguments[2].int_value).c_str());
                } else if (type == "suback") {
                } else if (type == "publish" && arguments.size() >= 7) {
                    auto topic = arguments[3].string_value;
                    if (arguments[4].int_value == arguments[5].int_value) {
                        if (on_message_callback_) {
                            on_message_callback_(topic, at_uart_->DecodeHex(arguments[6].string_value));
                        }
                    } else {
                        message_payload_.append(at_uart_->DecodeHex(arguments[6].string_value));
                        if (message_payload_.size() >= arguments[4].int_value && on_message_callback_) {
                            on_message_callback_(topic, message_payload_);
                            message_payload_.clear();
                        }
                    }
                } else {
                    ESP_LOGI(TAG, "unhandled MQTT event: %s", type.c_str());
                }
            }
        } else if (command == "MQTTSTATE" && arguments.size() == 1) {
            connected_ = arguments[0].int_value != 3;
            xEventGroupSetBits(event_group_handle_, MQTT_INITIALIZED_EVENT);
        }
    });
}

Ml307Mqtt::~Ml307Mqtt() {
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    vEventGroupDelete(event_group_handle_);
}

bool Ml307Mqtt::Connect(const std::string broker_address, int broker_port, const std::string client_id, const std::string username, const std::string password) {
    EventBits_t bits;
    if (IsConnected()) {
        // 断开之前的连接
        Disconnect();
        bits = xEventGroupWaitBits(event_group_handle_, MQTT_DISCONNECTED_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(MQTT_CONNECT_TIMEOUT_MS));
        if (!(bits & MQTT_DISCONNECTED_EVENT)) {
            ESP_LOGE(TAG, "Failed to disconnect from previous connection");
            return false;
        }
    }

    if (broker_port == 8883) {
        if (!at_uart_->SendCommand(std::string("AT+MQTTCFG=\"ssl\",") + std::to_string(mqtt_id_) + ",1")) {
            ESP_LOGE(TAG, "Failed to set MQTT to use SSL");
            return false;
        }
    }

    // Set clean session
    if (!at_uart_->SendCommand(std::string("AT+MQTTCFG=\"clean\",") + std::to_string(mqtt_id_) + ",1")) {
        ESP_LOGE(TAG, "Failed to set MQTT clean session");
        return false;
    }

    // Set keep alive and ping interval both to the same value
    if (!at_uart_->SendCommand(std::string("AT+MQTTCFG=\"keepalive\",") + std::to_string(mqtt_id_) + "," + std::to_string(keep_alive_seconds_))) {
        ESP_LOGE(TAG, "Failed to set MQTT keepalive interval");
        return false;
    }
    if (!at_uart_->SendCommand(std::string("AT+MQTTCFG=\"pingreq\",") + std::to_string(mqtt_id_) + "," + std::to_string(keep_alive_seconds_))) {
        ESP_LOGE(TAG, "Failed to set MQTT ping interval");
        return false;
    }

    // Set HEX encoding (ASCII for sending, HEX for receiving)
    if (!at_uart_->SendCommand("AT+MQTTCFG=\"encoding\"," + std::to_string(mqtt_id_) + ",0,1")) {
        ESP_LOGE(TAG, "Failed to set MQTT to use HEX encoding");
        return false;
    }

    // 创建MQTT连接
    std::string command = "AT+MQTTCONN=" + std::to_string(mqtt_id_) + ",\"" + broker_address + "\"," + std::to_string(broker_port) + ",\"" + client_id + "\",\"" + username + "\",\"" + password + "\"";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to create MQTT connection");
        return false;
    }

    // 等待连接完成
    bits = xEventGroupWaitBits(event_group_handle_, MQTT_CONNECTED_EVENT | MQTT_DISCONNECTED_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(MQTT_CONNECT_TIMEOUT_MS));
    if (!(bits & MQTT_CONNECTED_EVENT)) {
        ESP_LOGE(TAG, "Failed to connect to MQTT broker");
        return false;
    }

    if (on_connected_callback_) {
        on_connected_callback_();
    }
    return true;
}

bool Ml307Mqtt::IsConnected() {
    // 检查这个 id 是否已经连接
    at_uart_->SendCommand(std::string("AT+MQTTSTATE=") + std::to_string(mqtt_id_));
    auto bits = xEventGroupWaitBits(event_group_handle_, MQTT_INITIALIZED_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(MQTT_CONNECT_TIMEOUT_MS));
    if (!(bits & MQTT_INITIALIZED_EVENT)) {
        ESP_LOGE(TAG, "Failed to initialize MQTT connection");
        return false;
    }
    return connected_;
}

void Ml307Mqtt::Disconnect() {
    if (!connected_) {
        return;
    }
    at_uart_->SendCommand(std::string("AT+MQTTDISC=") + std::to_string(mqtt_id_));
}

bool Ml307Mqtt::Publish(const std::string topic, const std::string payload, int qos) {
    if (!connected_) {
        return false;
    }
    // If payload size is larger than 64KB, a CME ERROR 601 will be returned.
    std::string command = "AT+MQTTPUB=" + std::to_string(mqtt_id_) + ",\"" + topic + "\",";
    command += std::to_string(qos) + ",0,0,";
    command += std::to_string(payload.size());
    if (!at_uart_->SendCommand(command)) {
        return false;
    }
    return at_uart_->SendCommand(payload);
}

bool Ml307Mqtt::Subscribe(const std::string topic, int qos) {
    if (!connected_) {
        return false;
    }
    std::string command = "AT+MQTTSUB=" + std::to_string(mqtt_id_) + ",\"" + topic + "\"," + std::to_string(qos);
    return at_uart_->SendCommand(command);
}

bool Ml307Mqtt::Unsubscribe(const std::string topic) {
    if (!connected_) {
        return false;
    }
    std::string command = "AT+MQTTUNSUB=" + std::to_string(mqtt_id_) + ",\"" + topic + "\"";
    return at_uart_->SendCommand(command);
}

std::string Ml307Mqtt::ErrorToString(int error_code) {
    switch (error_code) {
        case 0:
            return "连接成功";
        case 1:
            return "正在重连";
        case 2:
            return "断开：用户主动断开";
        case 3:
            return "断开：拒绝连接（协议版本、标识符、用户名或密码错误）";
        case 4:
            return "断开：服务器断开";
        case 5:
            return "断开：Ping包超时断开";
        case 6:
            return "断开：网络异常断开";
        case 255:
            return "断开：未知错误";
        default:
            return "未知错误";
    }
}
