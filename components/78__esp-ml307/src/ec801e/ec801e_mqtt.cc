#include "ec801e_mqtt.h"
#include <esp_log.h>

#define TAG "Ec801EMqtt"

Ec801EMqtt::Ec801EMqtt(std::shared_ptr<AtUart> at_uart, int mqtt_id) : at_uart_(at_uart), mqtt_id_(mqtt_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "QMTRECV" && arguments.size() >= 4) {
            if (arguments[0].int_value == mqtt_id_) {
                auto topic = arguments[2].string_value;
                if (on_message_callback_) {
                    on_message_callback_(topic, at_uart_->DecodeHex(arguments[3].string_value));
                }
            }
        } else if (command == "QMTSTAT" && arguments.size() == 1) {
            if (arguments[0].int_value == mqtt_id_) {
                ESP_LOGI(TAG, "MQTT connection state: %s", ErrorToString(arguments[1].int_value).c_str());
            }
        } else if (command == "QMTCONN" && arguments.size() == 3) {
            if (arguments[0].int_value == mqtt_id_) {
                error_code_ = arguments[2].int_value;
                if (error_code_ == 0) {
                    connected_ = true;
                    xEventGroupSetBits(event_group_handle_, EC801E_MQTT_CONNECTED_EVENT);
                } else {
                    if (connected_) {
                        connected_ = false;
                        if (on_disconnected_callback_) {
                            on_disconnected_callback_();
                        }
                    }
                    xEventGroupSetBits(event_group_handle_, EC801E_MQTT_DISCONNECTED_EVENT);
                }
            }
        } else if (command == "QMTOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == mqtt_id_) {
                error_code_ = arguments[1].int_value;
                if (error_code_ == 0) {
                    xEventGroupSetBits(event_group_handle_, EC801E_MQTT_OPEN_COMPLETE);
                } else {
                    xEventGroupSetBits(event_group_handle_, EC801E_MQTT_OPEN_FAILED);
                }
            }
        } else if (command == "QMTDISC" && arguments.size() == 2) {
            if (arguments[0].int_value == mqtt_id_) {
                if (arguments[1].int_value == 0) {
                    xEventGroupSetBits(event_group_handle_, EC801E_MQTT_DISCONNECTED_EVENT);
                } else {
                    ESP_LOGE(TAG, "Failed to disconnect from MQTT broker");
                }
            }
        }
    });
}

Ec801EMqtt::~Ec801EMqtt() {
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    vEventGroupDelete(event_group_handle_);
}

bool Ec801EMqtt::Connect(const std::string broker_address, int broker_port, const std::string client_id, const std::string username, const std::string password) {
    EventBits_t bits;
    if (IsConnected()) {
        // 断开之前的连接
        Disconnect();
        bits = xEventGroupWaitBits(event_group_handle_, EC801E_MQTT_DISCONNECTED_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(EC801E_MQTT_CONNECT_TIMEOUT_MS));
        if (!(bits & EC801E_MQTT_DISCONNECTED_EVENT)) {
            ESP_LOGE(TAG, "Failed to disconnect from previous connection");
            return false;
        }
    }

    if (broker_port == 8883) {
        // Config SSL Context
        at_uart_->SendCommand("AT+QSSLCFG=\"sslversion\",2,4;+QSSLCFG=\"ciphersuite\",2,0xFFFF;+QSSLCFG=\"seclevel\",2,0");
        if (!at_uart_->SendCommand(std::string("AT+QMTCFG=\"ssl\",") + std::to_string(mqtt_id_) + ",1,2")) {
            ESP_LOGE(TAG, "Failed to set MQTT to use SSL");
            return false;
        }
    }

    // Set version
    if (!at_uart_->SendCommand(std::string("AT+QMTCFG=\"version\",") + std::to_string(mqtt_id_) + ",4")) {
        ESP_LOGE(TAG, "Failed to set MQTT version to 3.1.1");
        return false;
    }

    // Set clean session
    if (!at_uart_->SendCommand(std::string("AT+QMTCFG=\"session\",") + std::to_string(mqtt_id_) + ",1")) {
        ESP_LOGE(TAG, "Failed to set MQTT clean session");
        return false;
    }

    // Set keep alive
    if (!at_uart_->SendCommand(std::string("AT+QMTCFG=\"keepalive\",") + std::to_string(mqtt_id_) + "," + std::to_string(keep_alive_seconds_))) {
        ESP_LOGE(TAG, "Failed to set MQTT keep alive");
        return false;
    }

    // Set HEX encoding (ASCII for sending, HEX for receiving)
    if (!at_uart_->SendCommand("AT+QMTCFG=\"dataformat\"," + std::to_string(mqtt_id_) + ",0,1")) {
        ESP_LOGE(TAG, "Failed to set MQTT to use HEX encoding");
        return false;
    }

    std::string command = "AT+QMTOPEN=" + std::to_string(mqtt_id_) + ",\"" + broker_address + "\"," + std::to_string(broker_port);
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open MQTT connection");
        return false;
    }

    bits = xEventGroupWaitBits(event_group_handle_, EC801E_MQTT_OPEN_COMPLETE | EC801E_MQTT_OPEN_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(EC801E_MQTT_CONNECT_TIMEOUT_MS));
    if (bits & EC801E_MQTT_OPEN_FAILED) {
        const char* error_code_str[] = {
            "打开网络成功",
            "参数错误",
            "MQTT 标识符被占用",
            "激活 PDP 失败",
            "域名解析失败",
            "网络断开导致错误"
        };
        const char* message = error_code_ < 6 ? error_code_str[error_code_] : "未知错误";
        ESP_LOGE(TAG, "Failed to open MQTT connection: %s", message);

        if (error_code_ == 2) { // MQTT 标识符被占用
            connected_ = true;
            return Connect(broker_address, broker_port, client_id, username, password);
        }
        return false;
    } else if (!(bits & EC801E_MQTT_OPEN_COMPLETE)) {
        ESP_LOGE(TAG, "MQTT connection timeout");
        return false;
    }

    command = "AT+QMTCONN=" + std::to_string(mqtt_id_) + ",\"" + client_id + "\",\"" + username + "\",\"" + password + "\"";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to connect to MQTT broker");
        return false;
    }

    // 等待连接完成
    bits = xEventGroupWaitBits(event_group_handle_, EC801E_MQTT_CONNECTED_EVENT | EC801E_MQTT_DISCONNECTED_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(EC801E_MQTT_CONNECT_TIMEOUT_MS));
    if (bits & EC801E_MQTT_DISCONNECTED_EVENT) {
        const char* error_code_str[] = {
            "接受连接",
            "拒绝连接：不接受的协议版本",
            "拒绝连接：标识符被拒绝",
            "拒绝连接：服务器不可用",
            "拒绝连接：错误的用户名或密码",
            "拒绝连接：未授权"
        };
        const char* message = error_code_ < 6 ? error_code_str[error_code_] : "未知错误";
        ESP_LOGE(TAG, "Failed to connect to MQTT broker: %s", message);
        return false;
    } else if (!(bits & EC801E_MQTT_CONNECTED_EVENT)) {
        ESP_LOGE(TAG, "MQTT connection timeout");
        return false;
    }

    if (on_connected_callback_) {
        on_connected_callback_();
    }
    return true;
}

bool Ec801EMqtt::IsConnected() {
    return connected_;
}

void Ec801EMqtt::Disconnect() {
    if (!connected_) {
        return;
    }
    at_uart_->SendCommand(std::string("AT+QMTDISC=") + std::to_string(mqtt_id_));
}

bool Ec801EMqtt::Publish(const std::string topic, const std::string payload, int qos) {
    if (!connected_) {
        return false;
    }
    // If payload size is larger than 64KB, a CME ERROR 601 will be returned.
    std::string command = "AT+QMTPUBEX=" + std::to_string(mqtt_id_) + ",0,0,0,\"" + topic + "\",";
    command += std::to_string(payload.size());
    if (!at_uart_->SendCommandWithData(command, 1000, true, payload.data(), payload.size())) {
        return false;
    }
    return true;
}

bool Ec801EMqtt::Subscribe(const std::string topic, int qos) {
    if (!connected_) {
        return false;
    }
    std::string command = "AT+QMTSUB=" + std::to_string(mqtt_id_) + ",0,\"" + topic + "\"," + std::to_string(qos);
    return at_uart_->SendCommand(command);
}

bool Ec801EMqtt::Unsubscribe(const std::string topic) {
    if (!connected_) {
        return false;
    }
    std::string command = "AT+QMTUNS=" + std::to_string(mqtt_id_) + ",0,\"" + topic + "\"";
    return at_uart_->SendCommand(command);
}

std::string Ec801EMqtt::ErrorToString(int error_code) {
    switch (error_code) {
        case 0:
            return "连接成功";
        case 1:
            return "连接被服务器断开或者重置";
        case 2:
            return "发送 PINGREQ 包超时或者失败";
        case 3:
            return "发送 CONNECT 包超时或者失败";
        case 4:
            return "接收 CONNACK 包超时或者失败";
        case 5:
            return "客户端向服务器发送 DISCONNECT 包，但是服务器主动断开 MQTT 连接";
        case 6:
            return "因为发送数据包总是失败，客户端主动断开 MQTT 连接";
        case 7:
            return "链路不工作或者服务器不可用";
        case 8:
            return "客户端主动断开 MQTT 连接";
        default:
            return "未知错误";
    }
}
