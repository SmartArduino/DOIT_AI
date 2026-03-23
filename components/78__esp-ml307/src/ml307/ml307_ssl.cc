#include "ml307_ssl.h"
#include <esp_log.h>

static const char *TAG = "Ml307Ssl";

Ml307Ssl::Ml307Ssl(std::shared_ptr<AtUart> at_uart, int tcp_id) : Ml307Tcp(at_uart, tcp_id) {
}

bool Ml307Ssl::ConfigureSsl(int port) {
    // 设置 SSL 配置
    std::string command = "AT+MSSLCFG=\"auth\",0,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set SSL configuration");
        return false;
    }

    // 强制启用 SSL
    command = "AT+MIPCFG=\"ssl\"," + std::to_string(tcp_id_) + ",1,0";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to set SSL configuration");
        return false;
    }

    return true;
} 