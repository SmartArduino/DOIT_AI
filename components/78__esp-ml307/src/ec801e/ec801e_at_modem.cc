#include "ec801e_at_modem.h"
#include <esp_log.h>
#include <esp_err.h>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <cstring>
#include "ec801e_ssl.h"
#include "ec801e_tcp.h"
#include "ec801e_udp.h"
#include "ec801e_mqtt.h"
#include "http_client.h"
#include "web_socket.h"

#define TAG "Ec801EAtModem"


Ec801EAtModem::Ec801EAtModem(std::shared_ptr<AtUart> at_uart) : AtModem(at_uart) {
    // 子类特定的初始化在这里
    // ATE0 关闭 echo
    at_uart_->SendCommand("ATE0");
    // 设置 URC 端口为 UART1
    at_uart_->SendCommand("AT+QURCCFG=\"urcport\",\"uart1\"");
}

void Ec801EAtModem::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    // Handle Common URC
    AtModem::HandleUrc(command, arguments);
}

bool Ec801EAtModem::SetSleepMode(bool enable, int delay_seconds) {
    if (enable) {
        if (delay_seconds > 0) {
            at_uart_->SendCommand("AT+QSCLKEX=1," + std::to_string(delay_seconds) + ",30");
        }
        return at_uart_->SendCommand("AT+QSCLK=1");
    } else {
        return at_uart_->SendCommand("AT+QSCLK=0");
    }
}

std::unique_ptr<Http> Ec801EAtModem::CreateHttp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<HttpClient>(this, connect_id);
}

std::unique_ptr<Tcp> Ec801EAtModem::CreateTcp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801ETcp>(at_uart_, connect_id);
}

std::unique_ptr<Tcp> Ec801EAtModem::CreateSsl(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801ESsl>(at_uart_, connect_id);
}

std::unique_ptr<Udp> Ec801EAtModem::CreateUdp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801EUdp>(at_uart_, connect_id);
}

std::unique_ptr<Mqtt> Ec801EAtModem::CreateMqtt(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801EMqtt>(at_uart_, connect_id);
}

std::unique_ptr<WebSocket> Ec801EAtModem::CreateWebSocket(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<WebSocket>(this, connect_id);
}