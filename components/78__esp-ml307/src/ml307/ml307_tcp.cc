#include "ml307_tcp.h"
#include <esp_log.h>
#include <cstring>

#define TAG "Ml307Tcp"

Ml307Tcp::Ml307Tcp(std::shared_ptr<AtUart> at_uart, int tcp_id) : at_uart_(at_uart), tcp_id_(tcp_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "MIPOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == tcp_id_) {
                connected_ = arguments[1].int_value == 0;
                if (connected_) {
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, ML307_TCP_DISCONNECTED | ML307_TCP_ERROR);
                    xEventGroupSetBits(event_group_handle_, ML307_TCP_CONNECTED);
                } else {
                    xEventGroupSetBits(event_group_handle_, ML307_TCP_ERROR);
                }
            }
        } else if (command == "MIPCLOSE" && arguments.size() == 1) {
            if (arguments[0].int_value == tcp_id_) {
                instance_active_ = false;
                xEventGroupSetBits(event_group_handle_, ML307_TCP_DISCONNECTED);
            }
        } else if (command == "MIPSEND" && arguments.size() == 2) {
            if (arguments[0].int_value == tcp_id_) {
                xEventGroupSetBits(event_group_handle_, ML307_TCP_SEND_COMPLETE);
            }
        } else if (command == "MIPURC" && arguments.size() >= 3) {
            if (arguments[1].int_value == tcp_id_) {
                if (arguments[0].string_value == "rtcp") {
                    if (connected_ && stream_callback_) {
                        stream_callback_(at_uart_->DecodeHex(arguments[3].string_value));
                    }
                } else if (arguments[0].string_value == "disconn") {
                    if (connected_) {
                        connected_ = false;
                        if (disconnect_callback_) {
                            disconnect_callback_();
                        }
                    }
                    instance_active_ = false;
                    xEventGroupSetBits(event_group_handle_, ML307_TCP_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown MIPURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "MIPSTATE" && arguments.size() >= 5) {
            if (arguments[0].int_value == tcp_id_) {
                connected_ = arguments[4].string_value == "CONNECTED";
                instance_active_ = arguments[4].string_value != "INITIAL";
                xEventGroupSetBits(event_group_handle_, ML307_TCP_INITIALIZED);
            }
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, ML307_TCP_ERROR);
            Disconnect();
        }
    });
}

Ml307Tcp::~Ml307Tcp() {
    Disconnect();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool Ml307Tcp::Connect(const std::string& host, int port) {
    // Clear bits
    xEventGroupClearBits(event_group_handle_, ML307_TCP_CONNECTED | ML307_TCP_DISCONNECTED | ML307_TCP_ERROR);

    // 检查这个 id 是否已经连接
    std::string command = "AT+MIPSTATE=" + std::to_string(tcp_id_);
    at_uart_->SendCommand(command);
    auto bits = xEventGroupWaitBits(event_group_handle_, ML307_TCP_INITIALIZED, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
    if (!(bits & ML307_TCP_INITIALIZED)) {
        ESP_LOGE(TAG, "Failed to initialize TCP connection");
        return false;
    }

    // 断开之前的连接
    if (instance_active_) {
        command = "AT+MIPCLOSE=" + std::to_string(tcp_id_);
        if (at_uart_->SendCommand(command)) {
            // 等待断开完成
            xEventGroupWaitBits(event_group_handle_, ML307_TCP_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
        }
    }

    // 配置SSL（子类可以重写）
    if (!ConfigureSsl(port)) {
        ESP_LOGE(TAG, "Failed to configure SSL");
        return false;
    }

    // 使用 HEX 编码
    command = "AT+MIPCFG=\"encoding\"," + std::to_string(tcp_id_) + ",1,1";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set HEX encoding");
        return false;
    }

    // 打开 TCP 连接
    command = "AT+MIPOPEN=" + std::to_string(tcp_id_) + ",\"TCP\",\"" + host + "\"," + std::to_string(port) + ",,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open TCP connection, error=%d", at_uart_->GetCmeErrorCode());
        return false;
    }

    // 等待连接完成
    bits = xEventGroupWaitBits(event_group_handle_, ML307_TCP_CONNECTED | ML307_TCP_ERROR, pdTRUE, pdFALSE, TCP_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (bits & ML307_TCP_ERROR) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        return false;
    }
    return true;
}

void Ml307Tcp::Disconnect() {
    if (!instance_active_) {
        return;
    }
    
    std::string command = "AT+MIPCLOSE=" + std::to_string(tcp_id_);
    if (at_uart_->SendCommand(command)) {
        xEventGroupWaitBits(event_group_handle_, ML307_TCP_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
    }

    if (connected_) {
        connected_ = false;
        if (disconnect_callback_) {
            disconnect_callback_();
        }
    }
}

bool Ml307Tcp::ConfigureSsl(int port) {
    std::string command = "AT+MIPCFG=\"ssl\"," + std::to_string(tcp_id_) + ",0,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set SSL configuration");
        return false;
    }
    return true;
}

int Ml307Tcp::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460 / 2;
    size_t total_sent = 0;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    // 在循环外预先分配command
    std::string command;
    command.reserve(32 + MAX_PACKET_SIZE * 2);  // 预分配最大可能需要的空间

    while (total_sent < data.size()) {
        size_t chunk_size = std::min(data.size() - total_sent, MAX_PACKET_SIZE);
        
        // 重置command并构建新的命令，利用预分配的容量
        command.clear();
        command += "AT+MIPSEND=";
        command += std::to_string(tcp_id_);
        command += ",";
        command += std::to_string(chunk_size);
        command += ",";
        
        // 直接在command字符串上进行十六进制编码
        at_uart_->EncodeHexAppend(command, data.data() + total_sent, chunk_size);
        command += "\r\n";
        
        if (!at_uart_->SendCommand(command, 100, false)) {
            ESP_LOGE(TAG, "Failed to send data chunk");
            Disconnect();
            return -1;
        }

        auto bits = xEventGroupWaitBits(event_group_handle_, ML307_TCP_SEND_COMPLETE, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
        if (!(bits & ML307_TCP_SEND_COMPLETE)) {
            ESP_LOGE(TAG, "No send confirmation received");
            return -1;
        }

        total_sent += chunk_size;
    }
    return data.size();
}
