#include "ml307_udp.h"

#include <esp_log.h>

#define TAG "Ml307Udp"


Ml307Udp::Ml307Udp(std::shared_ptr<AtUart> at_uart, int udp_id) : at_uart_(at_uart), udp_id_(udp_id) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "MIPOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == udp_id_) {
                connected_ = arguments[1].int_value == 0;
                if (connected_) {
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, ML307_UDP_DISCONNECTED | ML307_UDP_ERROR);
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_CONNECTED);
                } else {
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_ERROR);
                }
            }
        } else if (command == "MIPCLOSE" && arguments.size() == 1) {
            if (arguments[0].int_value == udp_id_) {
                instance_active_ = false;
                xEventGroupSetBits(event_group_handle_, ML307_UDP_DISCONNECTED);
            }
        } else if (command == "MIPSEND" && arguments.size() == 2) {
            if (arguments[0].int_value == udp_id_) {
                xEventGroupSetBits(event_group_handle_, ML307_UDP_SEND_COMPLETE);
            }
        } else if (command == "MIPURC" && arguments.size() == 4) {
            if (arguments[1].int_value == udp_id_) {
                if (arguments[0].string_value == "rudp") {
                    if (connected_ && message_callback_) {
                        message_callback_(at_uart_->DecodeHex(arguments[3].string_value));
                    }
                } else if (arguments[0].string_value == "disconn") {
                    connected_ = false;
                    instance_active_ = false;
                    xEventGroupSetBits(event_group_handle_, ML307_UDP_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown MIPURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "MIPSTATE" && arguments.size() == 5) {
            if (arguments[0].int_value == udp_id_) {
                connected_ = arguments[4].string_value == "CONNECTED";
                instance_active_ = arguments[4].string_value != "INITIAL";
                xEventGroupSetBits(event_group_handle_, ML307_UDP_INITIALIZED);
            }
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, ML307_UDP_ERROR);
            Disconnect();
        }
    });
}

Ml307Udp::~Ml307Udp() {
    Disconnect();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool Ml307Udp::Connect(const std::string& host, int port) {
    // Clear bits
    xEventGroupClearBits(event_group_handle_, ML307_UDP_CONNECTED | ML307_UDP_DISCONNECTED | ML307_UDP_ERROR);

    // 检查这个 id 是否已经连接
    std::string command = "AT+MIPSTATE=" + std::to_string(udp_id_);
    at_uart_->SendCommand(command);
    auto bits = xEventGroupWaitBits(event_group_handle_, ML307_UDP_INITIALIZED, pdTRUE, pdFALSE, pdMS_TO_TICKS(UDP_CONNECT_TIMEOUT_MS));
    if (!(bits & ML307_UDP_INITIALIZED)) {
        ESP_LOGE(TAG, "Failed to initialize TCP connection");
        return false;
    }

    // 断开之前的连接
    if (instance_active_) {
        command = "AT+MIPCLOSE=" + std::to_string(udp_id_);
        if (at_uart_->SendCommand(command)) {
            // 等待断开完成
            xEventGroupWaitBits(event_group_handle_, ML307_UDP_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(UDP_CONNECT_TIMEOUT_MS));
        }
    }

    // 使用 HEX 编码
    command = "AT+MIPCFG=\"encoding\"," + std::to_string(udp_id_) + ",1,1";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set HEX encoding");
        return false;
    }
    command = "AT+MIPCFG=\"ssl\"," + std::to_string(udp_id_) + ",0,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set SSL configuration");
        return false;
    }

    // 打开 UDP 连接
    command = "AT+MIPOPEN=" + std::to_string(udp_id_) + ",\"UDP\",\"" + host + "\"," + std::to_string(port) + ",,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open UDP connection");
        return false;
    }

    // 等待连接完成
    bits = xEventGroupWaitBits(event_group_handle_, ML307_UDP_CONNECTED | ML307_UDP_ERROR, pdTRUE, pdFALSE, UDP_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (bits & ML307_UDP_ERROR) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        return false;
    }
    return true;
}


void Ml307Udp::Disconnect() {
    if (!instance_active_) {
        return;
    }

    at_uart_->SendCommand("AT+MIPCLOSE=" + std::to_string(udp_id_));
    connected_ = false;
}

int Ml307Udp::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460 / 2;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    if (data.size() > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "Data chunk exceeds maximum limit");
        return -1;
    }

    // 在循环外预先分配command
    std::string command = "AT+MIPSEND=" + std::to_string(udp_id_) + "," + std::to_string(data.size()) + ",";
    
    // 直接在command字符串上进行十六进制编码
    at_uart_->EncodeHexAppend(command, data.data(), data.size());
    command += "\r\n";
    
    if (!at_uart_->SendCommand(command, 100, false)) {
        ESP_LOGE(TAG, "Failed to send data chunk");
        return -1;
    }
    return data.size();
}
