#include "ec801e_ssl.h"

#include <esp_log.h>

#define TAG "Ec801ESsl"


Ec801ESsl::Ec801ESsl(std::shared_ptr<AtUart> at_uart, int ssl_id) : at_uart_(at_uart), ssl_id_(ssl_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "QSSLOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == ssl_id_ && !instance_active_) {
                if (arguments[1].int_value == 0) {
                    connected_ = true;
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, EC801E_SSL_DISCONNECTED | EC801E_SSL_ERROR);
                    xEventGroupSetBits(event_group_handle_, EC801E_SSL_CONNECTED);
                } else {
                    connected_ = false;
                    xEventGroupSetBits(event_group_handle_, EC801E_SSL_ERROR);
                }
            }
        } else if (command == "QSSLCLOSE" && arguments.size() == 1) {
            if (arguments[0].int_value == ssl_id_) {
                instance_active_ = false;
            }
        } else if (command == "QISEND" && arguments.size() == 3) {
            if (arguments[0].int_value == ssl_id_) {
                if (arguments[1].int_value == 0) {
                    xEventGroupSetBits(event_group_handle_, EC801E_SSL_SEND_COMPLETE);
                } else {
                    xEventGroupSetBits(event_group_handle_, EC801E_SSL_ERROR);
                }
            }
        } else if (command == "QSSLURC" && arguments.size() >= 2) {
            if (arguments[1].int_value == ssl_id_) {
                if (arguments[0].string_value == "recv" && arguments.size() >= 4) {
                    if (stream_callback_) {
                        stream_callback_(at_uart_->DecodeHex(arguments[3].string_value));
                    }
                } else if (arguments[0].string_value == "closed") {
                    if (connected_) {
                        connected_ = false;
                        // instance_active_ 保持 true，需要发送 QICLOSE 清理
                        if (disconnect_callback_) {
                            disconnect_callback_();
                        }
                    }
                    xEventGroupSetBits(event_group_handle_, EC801E_SSL_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown QIURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "QSSLSTATE" && arguments.size() > 5) {
            if (arguments[0].int_value == ssl_id_) {
                connected_ = arguments[5].int_value == 2;
                instance_active_ = true;
                xEventGroupSetBits(event_group_handle_, EC801E_SSL_INITIALIZED);
            }
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, EC801E_SSL_ERROR);
            Disconnect();
        }
    });
}

Ec801ESsl::~Ec801ESsl() {
    Disconnect();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
}

bool Ec801ESsl::Connect(const std::string& host, int port) {
    // Clear bits
    xEventGroupClearBits(event_group_handle_, EC801E_SSL_CONNECTED | EC801E_SSL_DISCONNECTED | EC801E_SSL_ERROR);

    // Keep data in one line; Use HEX encoding in response
    at_uart_->SendCommand("AT+QICFG=\"close/mode\",1;+QICFG=\"viewmode\",1;+QICFG=\"sendinfo\",1;+QICFG=\"dataformat\",0,1");

    // Config SSL Context
    at_uart_->SendCommand("AT+QSSLCFG=\"sslversion\",1,4;+QSSLCFG=\"ciphersuite\",1,0xFFFF;+QSSLCFG=\"seclevel\",1,0");
    // at_uart_->SendCommand("AT+QSSLCFG=\"cacert\",1,\"UFS:cacert.pem\"");

    // 检查这个 id 是否已经连接
    std::string command = "AT+QSSLSTATE=1," + std::to_string(ssl_id_);
    at_uart_->SendCommand(command);

    // 断开之前的连接（不触发回调事件）
    if (instance_active_) {
        at_uart_->SendCommand("AT+QSSLCLOSE=" + std::to_string(ssl_id_));
        xEventGroupWaitBits(event_group_handle_, EC801E_SSL_DISCONNECTED, pdTRUE, pdFALSE, SSL_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
        instance_active_ = false;
    }

    // 打开 TCP 连接
    command = "AT+QSSLOPEN=1,1," + std::to_string(ssl_id_) + ",\"" + host + "\"," + std::to_string(port) + ",1";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open TCP connection");
        return false;
    }

    // 等待连接完成
    auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_SSL_CONNECTED | EC801E_SSL_ERROR, pdTRUE, pdFALSE, SSL_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (bits & EC801E_SSL_ERROR) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        return false;
    }
    return true;
}


void Ec801ESsl::Disconnect() {
    if (!instance_active_) {
        return;
    }
    
    at_uart_->SendCommand("AT+QSSLCLOSE=" + std::to_string(ssl_id_));

    if (connected_) {
        connected_ = false;
        if (disconnect_callback_) {
            disconnect_callback_();
        }
    }
}

int Ec801ESsl::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460;
    size_t total_sent = 0;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    while (total_sent < data.size()) {
        size_t chunk_size = std::min(data.size() - total_sent, MAX_PACKET_SIZE);
        
        std::string command = "AT+QSSLSEND=" + std::to_string(ssl_id_) + "," + std::to_string(chunk_size);
        
        if (!at_uart_->SendCommandWithData(command, 1000, true, data.data() + total_sent, chunk_size)) {
            ESP_LOGE(TAG, "Send command failed");
            Disconnect();
            return -1;
        }

        auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_SSL_SEND_COMPLETE | EC801E_SSL_SEND_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(SSL_CONNECT_TIMEOUT_MS));
        if (bits & EC801E_SSL_SEND_FAILED) {
            ESP_LOGE(TAG, "Send failed, retry later");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        } else if (!(bits & EC801E_SSL_SEND_COMPLETE)) {
            ESP_LOGE(TAG, "Send timeout");
            return -1;
        }
        
        total_sent += chunk_size;
    }
    return data.size();
}
