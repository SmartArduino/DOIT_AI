#include "ec801e_tcp.h"

#include <esp_log.h>

#define TAG "Ec801ETcp"


Ec801ETcp::Ec801ETcp(std::shared_ptr<AtUart> at_uart, int tcp_id) : at_uart_(at_uart), tcp_id_(tcp_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "QIOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == tcp_id_) {
                if (arguments[1].int_value == 0) {
                    connected_ = true;
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, EC801E_TCP_DISCONNECTED | EC801E_TCP_ERROR);
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_CONNECTED);
                } else {
                    connected_ = false;
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_ERROR);
                    if (disconnect_callback_) {
                        disconnect_callback_();
                    }
                }
            }
        } else if (command == "QISEND" && arguments.size() == 3) {
            if (arguments[0].int_value == tcp_id_) {
                if (arguments[1].int_value == 0) {
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_SEND_COMPLETE);
                } else {
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_SEND_FAILED);
                }
            }
        } else if (command == "QIURC" && arguments.size() >= 2) {
            if (arguments[1].int_value == tcp_id_) {
                if (arguments[0].string_value == "recv" && arguments.size() >= 4) {
                    if (connected_ && stream_callback_) {
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
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown QIURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "QISTATE" && arguments.size() > 5) {
            if (arguments[0].int_value == tcp_id_) {
                connected_ = arguments[5].int_value == 2;
                instance_active_ = true;
                xEventGroupSetBits(event_group_handle_, EC801E_TCP_INITIALIZED);
            }
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, EC801E_TCP_ERROR);
            Disconnect();
        }
    });
}

Ec801ETcp::~Ec801ETcp() {
    Disconnect();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool Ec801ETcp::Connect(const std::string& host, int port) {
    // Clear bits
    xEventGroupClearBits(event_group_handle_, EC801E_TCP_CONNECTED | EC801E_TCP_DISCONNECTED | EC801E_TCP_ERROR);

    // Keep data in one line; Use HEX encoding in response
    at_uart_->SendCommand("AT+QICFG=\"close/mode\",1;+QICFG=\"viewmode\",1;+QICFG=\"sendinfo\",1;+QICFG=\"dataformat\",0,1");

    // 检查这个 id 是否已经连接
    std::string command = "AT+QISTATE=1," + std::to_string(tcp_id_);
    at_uart_->SendCommand(command);

    // 断开之前的连接（不触发回调事件）
    if (instance_active_) {
        at_uart_->SendCommand("AT+QICLOSE=" + std::to_string(tcp_id_));
        xEventGroupWaitBits(event_group_handle_, EC801E_TCP_DISCONNECTED, pdTRUE, pdFALSE, TCP_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
        instance_active_ = false;
    }

    // 打开 TCP 连接
    command = "AT+QIOPEN=1," + std::to_string(tcp_id_) + ",\"TCP\",\"" + host + "\"," + std::to_string(port) + ",0,1";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open TCP connection");
        return false;
    }

    // 等待连接完成
    auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_TCP_CONNECTED | EC801E_TCP_ERROR, pdTRUE, pdFALSE, TCP_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (bits & EC801E_TCP_ERROR) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        return false;
    }
    return true;
}

void Ec801ETcp::Disconnect() {
    if (!instance_active_) {
        return;
    }
    
    if (at_uart_->SendCommand("AT+QICLOSE=" + std::to_string(tcp_id_))) {
        instance_active_ = false;
    }

    if (connected_) {
        connected_ = false;
        if (disconnect_callback_) {
            disconnect_callback_();
        }
    }
}

int Ec801ETcp::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460;
    size_t total_sent = 0;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    while (total_sent < data.size()) {
        size_t chunk_size = std::min(data.size() - total_sent, MAX_PACKET_SIZE);
        
        std::string command = "AT+QISEND=" + std::to_string(tcp_id_) + "," + std::to_string(chunk_size);
        
        if (!at_uart_->SendCommandWithData(command, 1000, true, data.data() + total_sent, chunk_size)) {
            ESP_LOGE(TAG, "Send command failed");
            Disconnect();
            return -1;
        }
        
        auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_TCP_SEND_COMPLETE | EC801E_TCP_SEND_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
        if (bits & EC801E_TCP_SEND_FAILED) {
            ESP_LOGE(TAG, "Send failed, retry later");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        } else if (!(bits & EC801E_TCP_SEND_COMPLETE)) {
            ESP_LOGE(TAG, "Send timeout");
            return -1;
        }
        
        total_sent += chunk_size;
    }
    return data.size();
}
