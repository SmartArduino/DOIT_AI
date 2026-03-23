#include "ml307_at_modem.h"
#include <esp_log.h>
#include <esp_err.h>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <cstring>
#include "ml307_tcp.h"
#include "ml307_ssl.h"
#include "ml307_udp.h"
#include "ml307_mqtt.h"
#include "ml307_http.h"
#include "web_socket.h"

#define TAG "Ml307AtModem"


Ml307AtModem::Ml307AtModem(std::shared_ptr<AtUart> at_uart) : AtModem(at_uart) {
    // 子类特定的初始化在这里
    // Reset HTTP instances
    ResetConnections();
}

void Ml307AtModem::ResetConnections() {
    at_uart_->SendCommand("AT+MHTTPDEL=0");
    at_uart_->SendCommand("AT+MHTTPDEL=1");
    at_uart_->SendCommand("AT+MHTTPDEL=2");
    at_uart_->SendCommand("AT+MHTTPDEL=3");
}

void Ml307AtModem::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    // Handle Common URC
    AtModem::HandleUrc(command, arguments);
    // Handle ML307 URC
    if (command == "MIPCALL" && arguments.size() >= 3) {
        if (arguments[1].int_value == 1) {
            auto ip = arguments[2].string_value;
            ESP_LOGI(TAG, "PDP Context %d IP: %s", arguments[0].int_value, ip.c_str());
            network_ready_ = true;
            xEventGroupSetBits(event_group_handle_, AT_EVENT_NETWORK_READY);
        }
    } else if (command == "MATREADY") {
        if (network_ready_) {
            network_ready_ = false;
            if (on_network_state_changed_) {
                on_network_state_changed_(false);
            }
        }
    }
}

void Ml307AtModem::Reboot() {
    at_uart_->SendCommand("AT+MREBOOT=0");
}

bool Ml307AtModem::SetSleepMode(bool enable, int delay_seconds) {
    if (enable) {
        if (delay_seconds > 0) {
            at_uart_->SendCommand("AT+MLPMCFG=\"delaysleep\"," + std::to_string(delay_seconds));
        }
        return at_uart_->SendCommand("AT+MLPMCFG=\"sleepmode\",2,0");
    } else {
        return at_uart_->SendCommand("AT+MLPMCFG=\"sleepmode\",0,0");
    }
}

NetworkStatus Ml307AtModem::WaitForNetworkReady(int timeout_ms) {
    NetworkStatus status = AtModem::WaitForNetworkReady(timeout_ms);
    if (status == NetworkStatus::Ready) {
        // Wait for IP address, maximum total wait time is 4270ms
        int delay_ms = 10;
        for (int i = 0; i < 10; i++) {
            at_uart_->SendCommand("AT+MIPCALL?");
            auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_NETWORK_READY, pdFALSE, pdTRUE, pdMS_TO_TICKS(delay_ms));
            if (bits & AT_EVENT_NETWORK_READY) {
                return NetworkStatus::Ready;
            }
            delay_ms = std::min(delay_ms * 2, 1000);
        }
        ESP_LOGE(TAG, "Network ready but no IP address");
    }
    return status;
}

std::unique_ptr<Http> Ml307AtModem::CreateHttp(int connect_id) {
    return std::make_unique<Ml307Http>(at_uart_);
}

std::unique_ptr<Tcp> Ml307AtModem::CreateTcp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ml307Tcp>(at_uart_, connect_id);
}

std::unique_ptr<Tcp> Ml307AtModem::CreateSsl(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ml307Ssl>(at_uart_, connect_id);
}

std::unique_ptr<Udp> Ml307AtModem::CreateUdp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ml307Udp>(at_uart_, connect_id);
}

std::unique_ptr<Mqtt> Ml307AtModem::CreateMqtt(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ml307Mqtt>(at_uart_, connect_id);
}

std::unique_ptr<WebSocket> Ml307AtModem::CreateWebSocket(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<WebSocket>(this, connect_id);
}
