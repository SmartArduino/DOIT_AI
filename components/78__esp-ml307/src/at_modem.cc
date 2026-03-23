#include "at_modem.h"
#include "ml307/ml307_at_modem.h"
#include "ec801e/ec801e_at_modem.h"
#include <esp_log.h>
#include <esp_err.h>
#include <sstream>
#include <iomanip>
#include <cstring>

static const char* TAG = "AtModem";

std::unique_ptr<AtModem> AtModem::Detect(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin, int baud_rate) {
    // 创建AtUart进行检测
    auto uart = std::make_shared<AtUart>(tx_pin, rx_pin, dtr_pin);
    uart->Initialize();
    
    // 设置波特率
    if (!uart->SetBaudRate(baud_rate)) {
        return nullptr;
    }
    
    // 发送AT+CGMR（或ATI）命令获取模组型号
    if (!uart->SendCommand("AT+CGMR", 3000)) {
        ESP_LOGE(TAG, "Failed to send AT+CGMR command");
        return nullptr;
    }
    
    std::string response = uart->GetResponse();
    ESP_LOGI(TAG, "Detected modem: %s", response.c_str());
    
    // 检查响应中的模组型号
    if (response.find("EC801E") == 0) {
        return std::make_unique<Ec801EAtModem>(uart);
    } else if (response.find("NT26K") == 0) {
        return std::make_unique<Ec801EAtModem>(uart);
    } else if (response.find("ML307") == 0) {
        return std::make_unique<Ml307AtModem>(uart);
    } else {
        ESP_LOGE(TAG, "Unrecognized modem type: %s, use ML307 AtModem as default", response.c_str());
        return std::make_unique<Ml307AtModem>(uart);
    }
}

AtModem::AtModem(std::shared_ptr<AtUart> at_uart) : at_uart_(at_uart) {
    event_group_handle_ = xEventGroupCreate();
    at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        HandleUrc(command, arguments);
    });
}

AtModem::~AtModem() {
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

void AtModem::OnNetworkStateChanged(std::function<void(bool network_ready)> callback) {
    on_network_state_changed_ = callback;
}

void AtModem::Reboot() {
}

void AtModem::SetFlightMode(bool enable) {
    if (enable) {
        at_uart_->SendCommand("AT+CFUN=4"); // flight mode
        at_uart_->SetDtrPin(enable);
        network_ready_ = false;
    } else {
        at_uart_->SetDtrPin(enable);
        at_uart_->SendCommand("AT+CFUN=1"); // normal mode
    }
}

bool AtModem::SetSleepMode(bool enable, int delay_seconds) {
    return false;
}

NetworkStatus AtModem::WaitForNetworkReady(int timeout_ms) {
    ESP_LOGI(TAG, "Waiting for network ready...");
    network_ready_ = false;
    cereg_state_ = CeregState{};
    xEventGroupClearBits(event_group_handle_, AT_EVENT_NETWORK_READY | AT_EVENT_NETWORK_ERROR);
    
    // 检查 SIM 卡是否准备好
    for (int i = 0; i < 10; i++) {
        if (at_uart_->SendCommand("AT+CPIN?")) {
            pin_ready_ = true;
            break;
        }
        if (at_uart_->GetCmeErrorCode() == 10) {
            pin_ready_ = false;
            return NetworkStatus::ErrorInsertPin;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 检查网络注册状态
    if (!at_uart_->SendCommand("AT+CEREG=2")) {
        return NetworkStatus::Error;
    }
    if (!at_uart_->SendCommand("AT+CEREG?")) {
        return NetworkStatus::Error;
    }
    
    TickType_t timeout = portMAX_DELAY;
    if (timeout_ms > 0) {
        timeout = pdMS_TO_TICKS(timeout_ms);
    }
    auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_NETWORK_READY | AT_EVENT_NETWORK_ERROR, pdTRUE, pdFALSE, timeout);
    if (bits & AT_EVENT_NETWORK_READY) {
        return NetworkStatus::Ready;
    } else if (bits & AT_EVENT_NETWORK_ERROR) {
        if (cereg_state_.stat == 3) {
            return NetworkStatus::ErrorRegistrationDenied;
        } else if (!pin_ready_) {
            return NetworkStatus::ErrorInsertPin;
        } else {
            return NetworkStatus::Error;
        }
    }
    return NetworkStatus::ErrorTimeout;
}

std::string AtModem::GetImei() {
    if (!imei_.empty()) {
        return imei_;
    }
    at_uart_->SendCommand("AT+CGSN=1");
    return imei_;
}

std::string AtModem::GetIccid() {
    at_uart_->SendCommand("AT+ICCID");
    return iccid_;
}

std::string AtModem::GetModuleRevision() {
    if (!module_revision_.empty()) {
        return module_revision_;
    }
    if (at_uart_->SendCommand("AT+CGMR")) {
        module_revision_ = at_uart_->GetResponse();
    }
    return module_revision_;
}

std::string AtModem::GetCarrierName() {
    at_uart_->SendCommand("AT+COPS?");
    return carrier_name_;
}

int AtModem::GetCsq() {
    at_uart_->SendCommand("AT+CSQ", 10);
    return csq_;
}

CeregState AtModem::GetRegistrationState() {
    at_uart_->SendCommand("AT+CEREG?");
    return cereg_state_;
}

void AtModem::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    if (command == "CGSN" && arguments.size() >= 1) {
        imei_ = arguments[0].string_value;
    } else if (command == "ICCID" && arguments.size() >= 1) {
        iccid_ = arguments[0].string_value;
    } else if (command == "COPS" && arguments.size() >= 4) {
        carrier_name_ = arguments[2].string_value;
    } else if (command == "CSQ" && arguments.size() >= 1) {
        csq_ = arguments[0].int_value;
    } else if (command == "CEREG" && arguments.size() >= 1) {
        cereg_state_ = CeregState{};
        if (arguments.size() == 1) {
            cereg_state_.stat = 0;
        } else if (arguments.size() >= 2) {
            int state_index = arguments[1].type == AtArgumentValue::Type::Int ? 1 : 0;
            cereg_state_.stat = arguments[state_index].int_value;
            if (arguments.size() >= state_index + 2) {
                cereg_state_.tac = arguments[state_index + 1].string_value;
                cereg_state_.ci = arguments[state_index + 2].string_value;
                if (arguments.size() >= state_index + 4) {
                    cereg_state_.AcT = arguments[state_index + 3].int_value;
                }
            }
        }

        bool new_network_ready = cereg_state_.stat == 1 || cereg_state_.stat == 5;
        if (new_network_ready != network_ready_) {
            network_ready_ = new_network_ready;
            if (on_network_state_changed_) {
                on_network_state_changed_(new_network_ready);
            }
        }
        if (new_network_ready) {
            xEventGroupSetBits(event_group_handle_, AT_EVENT_NETWORK_READY);
        } else if (cereg_state_.stat == 3) {
            xEventGroupSetBits(event_group_handle_, AT_EVENT_NETWORK_ERROR);
        }
    } else if (command == "CPIN" && arguments.size() >= 1) {
        if (arguments[0].string_value == "READY") {
            pin_ready_ = true;
        } else {
            pin_ready_ = false;
        }
    }
}
