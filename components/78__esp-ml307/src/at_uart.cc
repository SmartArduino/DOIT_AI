#include "at_uart.h"
#include "freertos/projdefs.h"
#include <esp_log.h>
#include <esp_err.h>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <sstream>

#define TAG "AtUart"


// AtUart 构造函数实现
AtUart::AtUart(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), dtr_pin_(dtr_pin), uart_num_(UART_NUM),
      baud_rate_(115200), initialized_(false),
      event_task_handle_(nullptr), event_queue_handle_(nullptr), event_group_handle_(nullptr) {
}

AtUart::~AtUart() {
    if (event_task_handle_) {
        vTaskDelete(event_task_handle_);
    }
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
    if (initialized_) {
        uart_driver_delete(uart_num_);
    }
}

void AtUart::Initialize() {
    if (initialized_) {
        return;
    }
    
    event_group_handle_ = xEventGroupCreate();
    if (!event_group_handle_) {
        ESP_LOGE(TAG, "创建事件组失败");
        return;
    }

    gpio_set_direction(tx_pin_, GPIO_MODE_OUTPUT);
    gpio_set_direction(rx_pin_, GPIO_MODE_INPUT);
    gpio_set_level(tx_pin_, 1);
    gpio_set_level(rx_pin_, 1);
    gpio_set_pull_mode(tx_pin_, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(rx_pin_, GPIO_PULLUP_ONLY);

    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate_;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 1024 * 40, 0, 150, &event_queue_handle_, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    if (dtr_pin_ != GPIO_NUM_NC) {
        gpio_config_t config = {};
        config.pin_bit_mask = (1ULL << dtr_pin_);
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&config);
        gpio_set_level(dtr_pin_, 0);
    }

    xTaskCreate([](void* arg) {
        auto ml307_at_modem = (AtUart*)arg;
        ml307_at_modem->EventTask();
        vTaskDelete(NULL);
    }, "modem_event", 4096, this, 16, &event_task_handle_);

    xTaskCreate([](void* arg) {
        auto ml307_at_modem = (AtUart*)arg;
        ml307_at_modem->ReceiveTask();
        vTaskDelete(NULL);
    }, "modem_receive", 4096 * 3, this, 16, &receive_task_handle_);
    initialized_ = true;
}

void AtUart::EventTaskWrapper(void* arg) {
    auto uart = static_cast<AtUart*>(arg);
    uart->EventTask();
    vTaskDelete(nullptr);
}

void AtUart::EventTask() {
    uart_event_t event;
    while (true) {
        if (xQueueReceive(event_queue_handle_, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type)
            {
            case UART_DATA:
                xEventGroupSetBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE);
                break;
            case UART_BREAK:
                ESP_LOGI(TAG, "break");
                break;
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "buffer full");
                break;
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "FIFO overflow - clearing buffer");
                rx_buffer_.clear();
                xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_ERROR);
                {
                    std::vector<UrcCallback> callbacks_copy;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        callbacks_copy.assign(urc_callbacks_.begin(), urc_callbacks_.end());
                    }
                    for (auto& callback : callbacks_copy) {
                        try {
                            AtArgumentValue err_value;
                            err_value.type = AtArgumentValue::Type::String;
                            err_value.string_value = "FIFO_OVERFLOW";
                            callback("SYSTEM_ERROR", {err_value});
                        } catch (...) {}
                    }
                }
                break;
            default:
                ESP_LOGE(TAG, "unknown event type: %d", event.type);
                break;
            }
        }
    }
}

void AtUart::ReceiveTask() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE, pdTRUE, pdFALSE, pdMS_TO_TICKS(20));
        if (bits & AT_EVENT_DATA_AVAILABLE) {
            size_t available;
            uart_get_buffered_data_len(uart_num_, &available);
            if (available > 0) {
                // Extend rx_buffer_ and read into buffer
                rx_buffer_.resize(rx_buffer_.size() + available);
                char* rx_buffer_ptr = &rx_buffer_[rx_buffer_.size() - available];
                uart_read_bytes(uart_num_, rx_buffer_ptr, available, portMAX_DELAY);
                while (ParseResponse()) {}
            }
        }
    }
}

static bool is_number(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit) && s.length() < 10;
}

bool AtUart::ParseResponse() {
    if (wait_for_response_ && !rx_buffer_.empty() && rx_buffer_[0] == '>') {
        rx_buffer_.erase(0, 1);
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_DONE);
        return true;
    }

    size_t buffer_size = rx_buffer_.size();
    auto end_pos = rx_buffer_.find("\r\n");
    if (end_pos == std::string::npos) {
        if (buffer_size > AT_UART_MAX_UNKNOWN_BUFFER_SIZE) {
            ESP_LOGW(TAG, "Buffer too large (%zu bytes) without CRLF, checking content", buffer_size);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer_.c_str(), std::min((size_t)AT_UART_BUFFER_DUMP_SIZE, buffer_size));
            ESP_LOG_BUFFER_CHAR(TAG, rx_buffer_.c_str(), std::min((size_t)AT_UART_BUFFER_DUMP_SIZE, buffer_size));
        }

        // 通用解决方案：处理可能缺少换行符的 URC 消息
        const char* urc_prefixes[] = {"+MHTTPURC", "+MIPURC", "+MIPOPEN"};
        bool found_prefix = false;

        for (const char* prefix : urc_prefixes) {
            size_t prefix_len = strlen(prefix);
            if (buffer_size >= prefix_len && memcmp(rx_buffer_.c_str(), prefix, prefix_len) == 0) {
                found_prefix = true;
                ESP_LOGD(TAG, "Found URC prefix %s in buffer (%zu bytes)", prefix, buffer_size);
                size_t next_cmd_start = std::string::npos;

                // 查找下一个命令的开始位置
                size_t pos_plus = rx_buffer_.find("\n+", 1);
                size_t pos_ok = rx_buffer_.find("\nOK");
                size_t pos_err = rx_buffer_.find("\nERROR");

                // 找到最近的下一个命令标记
                if (pos_plus != std::string::npos) {
                    next_cmd_start = pos_plus;
                }
                if (pos_ok != std::string::npos && pos_ok < next_cmd_start) {
                    next_cmd_start = pos_ok;
                }
                if (pos_err != std::string::npos && pos_err < next_cmd_start) {
                    next_cmd_start = pos_err;
                }

                ESP_LOGD(TAG, "Next command pos: +=%zu, OK=%zu, ERR=%zu",
                         pos_plus, pos_ok, pos_err);

                if (next_cmd_start != std::string::npos && next_cmd_start > 0) {
                    // 在下一个命令前插入换行符
                    rx_buffer_.insert(next_cmd_start, "\r\n");
                    end_pos = rx_buffer_.find("\r\n");
                    ESP_LOGD(TAG, "Inserted CRLF at pos %zu, new size: %zu", next_cmd_start, rx_buffer_.size());
                    break;
                } else if (buffer_size > prefix_len + AT_UART_MAX_URC_BUFFER_SIZE) {
                    // 对于大数据流（如音频），达到上限
                    ESP_LOGW(TAG, "URC buffer too large (%zu bytes) for prefix %s, clearing", buffer_size, prefix);
                    rx_buffer_.clear();
                    return false;
                } else {
                    // 没找到下一个命令，可能数据还在继续到达
                    ESP_LOGD(TAG, "No next command found, waiting for more data");
                    return false;
                }
            }
        }

        if (!found_prefix && buffer_size > AT_UART_MAX_UNKNOWN_BUFFER_SIZE) {
            ESP_LOGE(TAG, "Unknown data in buffer (%zu bytes), clearing", buffer_size);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer_.c_str(), std::min((size_t)AT_UART_BUFFER_DUMP_SIZE, buffer_size));
            rx_buffer_.clear();
            return false;
        }

        if (end_pos == std::string::npos) {
            return false;
        }
    }

    // Ignore empty lines
    if (end_pos == 0) {
        rx_buffer_.erase(0, 2);
        return true;
    }

    ESP_LOGD(TAG, "<< %.64s (%u bytes)", rx_buffer_.substr(0, end_pos).c_str(), end_pos);

    // Parse "+CME ERROR: 123,456,789"
    if (rx_buffer_[0] == '+') {
        std::string command, values;
        auto pos = rx_buffer_.find(": ");
        if (pos == std::string::npos || pos > end_pos) {
            command = rx_buffer_.substr(1, end_pos - 1);
        } else {
            command = rx_buffer_.substr(1, pos - 1);
            values = rx_buffer_.substr(pos + 2, end_pos - pos - 2);
        }
        rx_buffer_.erase(0, end_pos + 2);

        // Parse "string", int, int, ... into AtArgumentValue
        std::vector<AtArgumentValue> arguments;
        std::istringstream iss(values);
        std::string item;
        while (std::getline(iss, item, ',')) {
            AtArgumentValue argument;
            if (item.front() == '"') {
                argument.type = AtArgumentValue::Type::String;
                argument.string_value = item.substr(1, item.size() - 2);
            } else if (item.find(".") != std::string::npos) {
                argument.type = AtArgumentValue::Type::Double;
                argument.double_value = std::stod(item);
            } else if (is_number(item)) {
                argument.type = AtArgumentValue::Type::Int;
                argument.int_value = std::stoi(item);
                argument.string_value = std::move(item);
            } else {
                argument.type = AtArgumentValue::Type::String;
                argument.string_value = std::move(item);
            }
            arguments.push_back(argument);
        }

        HandleUrc(command, arguments);
        return true;
    } else if (rx_buffer_.size() >= 4 && rx_buffer_[0] == 'O' && rx_buffer_[1] == 'K' && rx_buffer_[2] == '\r' && rx_buffer_[3] == '\n') {
        rx_buffer_.erase(0, 4);
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_DONE);
        return true;
    } else if (rx_buffer_.size() >= 7 && rx_buffer_[0] == 'E' && rx_buffer_[1] == 'R' && rx_buffer_[2] == 'R' && rx_buffer_[3] == 'O' && rx_buffer_[4] == 'R' && rx_buffer_[5] == '\r' && rx_buffer_[6] == '\n') {
        rx_buffer_.erase(0, 7);
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_ERROR);
        return true;
    } else {
        std::string raw_line = rx_buffer_.substr(0, end_pos);
        std::vector<AtArgumentValue> raw_args;
        AtArgumentValue raw_arg;
        raw_arg.type = AtArgumentValue::Type::String;
        raw_arg.string_value = raw_line;
        raw_args.push_back(raw_arg);
        HandleUrc("RAW_LINE", raw_args);

        std::lock_guard<std::mutex> lock(mutex_);
        response_ = std::move(raw_line);
        rx_buffer_.erase(0, end_pos + 2);
        return true;
    }
    return false;
}

void AtUart::HandleCommand(const char* command) {
    // 这个函数现在主要用于向后兼容，大部分处理逻辑已经移到 ParseLine 中
    if (wait_for_response_) {
        response_.append(command);
        response_.append("\r\n");
    }
}

void AtUart::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    if (command == "CME ERROR") {
        cme_error_code_ = arguments[0].int_value;
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_ERROR);
        return;
    }

    // 复制回调列表，避免在持有锁的情况下调用回调（防止死锁）
    std::vector<UrcCallback> callbacks_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_copy.assign(urc_callbacks_.begin(), urc_callbacks_.end());
    }

    // 在锁外调用回调，允许回调内部访问需要锁的函数
    for (auto& callback : callbacks_copy) {
        try {
            callback(command, arguments);
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "URC callback exception: %s", e.what());
        } catch (...) {
            ESP_LOGE(TAG, "URC callback unknown exception");
        }
    }
}

bool AtUart::DetectBaudRate() {
    int baud_rates[] = {115200, 921600, 460800, 230400, 57600, 38400, 19200, 9600,4800};
    while (true) {
        ESP_LOGI(TAG, "Detecting baud rate...");
        for (size_t i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++) {
            int rate = baud_rates[i];
            uart_set_baudrate(uart_num_, rate);
            if (SendCommand("AT", 20)) {
                ESP_LOGI(TAG, "Detected baud rate: %d", rate);
                baud_rate_ = rate;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

bool AtUart::SetBaudRate(int new_baud_rate) {
    if (!DetectBaudRate()) {
        ESP_LOGE(TAG, "Failed to detect baud rate");
        return false;
    }
    if (new_baud_rate == baud_rate_) {
        return true;
    }
    // Set new baud rate
    if (!SendCommand(std::string("AT+IPR=") + std::to_string(new_baud_rate))) {
        ESP_LOGI(TAG, "Failed to set baud rate to %d", new_baud_rate);
        return false;
    }
    uart_set_baudrate(uart_num_, new_baud_rate);
    baud_rate_ = new_baud_rate;
    ESP_LOGI(TAG, "Set baud rate to %d", new_baud_rate);
    return true;
}

bool AtUart::SendData(const char* data, size_t length) {
    if (!initialized_) {
        ESP_LOGE(TAG, "UART未初始化");
        return false;
    }
    
    int ret = uart_write_bytes(uart_num_, data, length);
    if (ret < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", ret);
        return false;
    }
    return true;
}

bool AtUart::SendCommandWithData(const std::string& command, size_t timeout_ms, bool add_crlf, const char* data, size_t data_length) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    ESP_LOGD(TAG, ">> %.64s (%u bytes)", command.data(), command.length());

    xEventGroupClearBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR);
    wait_for_response_ = true;
    cme_error_code_ = 0;
    response_.clear();

    if (add_crlf) {
        if (!SendData((command + "\r\n").data(), command.length() + 2)) {
            return false;
        }
    } else {
        if (!SendData(command.data(), command.length())) {
            return false;
        }
    }
    if (timeout_ms > 0) {
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
        wait_for_response_ = false;
        if (!(bits & AT_EVENT_COMMAND_DONE)) {
            return false;
        }
    } else {
        wait_for_response_ = false;
    }

    if (data && data_length > 0) {
        wait_for_response_ = true;
        if (!SendData(data, data_length)) {
            return false;
        }
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
        wait_for_response_ = false;
        if (!(bits & AT_EVENT_COMMAND_DONE)) {
            return false;
        }
    }
    return true;
}

bool AtUart::SendCommand(const std::string& command, size_t timeout_ms, bool add_crlf) {
    return SendCommandWithData(command, timeout_ms, add_crlf, nullptr, 0);
}

std::list<UrcCallback>::iterator AtUart::RegisterUrcCallback(UrcCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    return urc_callbacks_.insert(urc_callbacks_.end(), callback);
}

void AtUart::UnregisterUrcCallback(std::list<UrcCallback>::iterator iterator) {
    std::lock_guard<std::mutex> lock(mutex_);
    urc_callbacks_.erase(iterator);
}

void AtUart::SetDtrPin(bool high) {
    if (dtr_pin_ != GPIO_NUM_NC) {
        ESP_LOGD(TAG, "Set DTR pin %d to %d", dtr_pin_, high ? 1 : 0);
        gpio_set_level(dtr_pin_, high ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static const char hex_chars[] = "0123456789ABCDEF";
// 辅助函数，将单个十六进制字符转换为对应的数值
inline uint8_t CharToHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;  // 对于无效输入，返回0
}

void AtUart::EncodeHexAppend(std::string& dest, const char* data, size_t length) {
    dest.reserve(dest.size() + length * 2 + 4);  // 预分配空间，多分配4个字节用于\r\n\0
    for (size_t i = 0; i < length; i++) {
        dest.push_back(hex_chars[(data[i] & 0xF0) >> 4]);
        dest.push_back(hex_chars[data[i] & 0x0F]);
    }
}

void AtUart::DecodeHexAppend(std::string& dest, const char* data, size_t length) {
    dest.reserve(dest.size() + length / 2 + 4);  // 预分配空间，多分配4个字节用于\r\n\0
    for (size_t i = 0; i < length; i += 2) {
        char byte = (CharToHex(data[i]) << 4) | CharToHex(data[i + 1]);
        dest.push_back(byte);
    }
}

std::string AtUart::EncodeHex(const std::string& data) {
    std::string encoded;
    EncodeHexAppend(encoded, data.c_str(), data.size());
    return encoded;
}

std::string AtUart::DecodeHex(const std::string& data) {
    std::string decoded;
    DecodeHexAppend(decoded, data.c_str(), data.size());
    return decoded;
}
