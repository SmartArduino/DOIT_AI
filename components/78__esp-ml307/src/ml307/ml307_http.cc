#include "ml307_http.h"
#include <esp_log.h>
#include <cstring>
#include <sstream>
#include <chrono>

static const char *TAG = "Ml307Http";

Ml307Http::Ml307Http(std::shared_ptr<AtUart> at_uart) : at_uart_(at_uart) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (!this->callback_enabled_.load()) {
            return;
        }
        this->HandleUrcCallback(command, arguments);
    });
}

int Ml307Http::Read(char* buffer, size_t buffer_size) {
    std::unique_lock<std::mutex> lock(mutex_);

    // 先检查条件，避免错过通知
    if (eof_ && body_.empty()) {
        return 0;
    }

    // 如果已有数据，立即返回，不等待
    if (!body_.empty()) {
        size_t bytes_to_read = std::min(body_.size(), buffer_size);
        std::memcpy(buffer, body_.data(), bytes_to_read);
        body_.erase(0, bytes_to_read);
        return bytes_to_read;
    }

    // 等待数据或 EOF
    auto timeout = std::chrono::milliseconds(timeout_ms_);
    bool received = cv_.wait_for(lock, timeout, [this] {
        return !body_.empty() || eof_;
    });

    if (!received) {
        ESP_LOGE(TAG, "Timeout waiting for HTTP content to be received");
        // 检查是否真的超时还是通知丢失
        if (!body_.empty()) {
            ESP_LOGW(TAG, "Spurious timeout detected - data available");
            size_t bytes_to_read = std::min(body_.size(), buffer_size);
            std::memcpy(buffer, body_.data(), bytes_to_read);
            body_.erase(0, bytes_to_read);
            return bytes_to_read;
        }
        return -1;
    }

    if (eof_ && body_.empty()) {
        return 0;
    }

    size_t bytes_to_read = std::min(body_.size(), buffer_size);
    std::memcpy(buffer, body_.data(), bytes_to_read);
    body_.erase(0, bytes_to_read);

    return bytes_to_read;
}

int Ml307Http::Write(const char* buffer, size_t buffer_size) {
    if (buffer_size == 0) { // FIXME: 模组好像不支持发送空数据
        std::string command = "AT+MHTTPCONTENT=" + std::to_string(http_id_) + ",0,2,\"0D0A\"";
        at_uart_->SendCommand(command);
        return 0;
    }
    std::string command = "AT+MHTTPCONTENT=" + std::to_string(http_id_) + ",1," + std::to_string(buffer_size);
    at_uart_->SendCommand(command);
    at_uart_->SendCommand(std::string(buffer, buffer_size));
    return buffer_size;
}

Ml307Http::~Ml307Http() {
    callback_enabled_ = false;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (instance_active_) {
        Close();
    }

    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    vEventGroupDelete(event_group_handle_);
}

void Ml307Http::HandleUrcCallback(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    if (command == "RAW_LINE") {
        if (arguments.empty() || pending_hex_chars_expected_ == 0) {
            return;
        }
        std::string line = arguments[0].string_value;
        if (!line.empty() && line.front() == '"') {
            line.erase(line.begin());
        }
        if (!line.empty() && line.back() == '"') {
            line.pop_back();
        }
        for (char c : line) {
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                (c >= 'a' && c <= 'f')) {
                pending_hex_data_.push_back(c);
            }
        }
        if (pending_hex_data_.size() < pending_hex_chars_expected_) {
            return;
        }

        std::string hex_payload = pending_hex_data_.substr(0, pending_hex_chars_expected_);
        pending_hex_data_.erase(0, pending_hex_chars_expected_);

        std::string decoded_data;
        at_uart_->DecodeHexAppend(decoded_data, hex_payload.c_str(), hex_payload.length());

        {
            std::lock_guard<std::mutex> lock(mutex_);
            body_.append(decoded_data);
            body_offset_ += decoded_data.size();
            if (response_chunked_) {
                eof_ = (pending_chunk_len_ == 0);
            } else {
                eof_ = (pending_total_len_ > 0) &&
                       ((pending_reported_offset_ + pending_chunk_len_) >= pending_total_len_);
            }
        }
        cv_.notify_one();

        pending_total_len_ = 0;
        pending_reported_offset_ = 0;
        pending_chunk_len_ = 0;
        pending_hex_chars_expected_ = 0;
        pending_hex_data_.clear();
        return;
    }

    if (command == "MHTTPURC") {
        if (arguments.size() < 2) {
            ESP_LOGE(TAG, "Invalid MHTTPURC args size: %u", (unsigned int)arguments.size());
            return;
        }
        if (arguments[1].int_value == http_id_) {
            auto& type = arguments[0].string_value;
            if (type == "header") {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    eof_ = false;
                    body_offset_ = 0;
                    body_.clear();
                }
                response_headers_.clear();
                response_chunked_ = false;
                if (arguments.size() >= 3) {
                    status_code_ = arguments[2].int_value;
                } else {
                    status_code_ = -1;
                }
                if (arguments.size() >= 5) {
                    ParseResponseHeaders(at_uart_->DecodeHex(arguments[4].string_value));
                } else {
                    ESP_LOGE(TAG, "Missing header");
                }
                xEventGroupSetBits(event_group_handle_, ML307_HTTP_EVENT_HEADERS_RECEIVED);
            } else if (type == "content") {
                std::string decoded_data;
                if (arguments.size() >= 6) {
                    at_uart_->DecodeHexAppend(decoded_data, arguments[5].string_value.c_str(), arguments[5].string_value.length());
                    pending_total_len_ = 0;
                    pending_reported_offset_ = 0;
                    pending_chunk_len_ = 0;
                    pending_hex_chars_expected_ = 0;
                    pending_hex_data_.clear();
                } else if (arguments.size() >= 5) {
                    pending_total_len_ = arguments[2].int_value;
                    pending_reported_offset_ = arguments[3].int_value;
                    pending_chunk_len_ = arguments[4].int_value;
                    pending_hex_chars_expected_ = (pending_chunk_len_ > 0) ? ((size_t)pending_chunk_len_ * 2U) : 0U;
                    pending_hex_data_.clear();
                    if (pending_hex_chars_expected_ > 0) {
                        ESP_LOGW(TAG, "Content payload moved to RAW_LINE, wait %u hex chars",
                                 (unsigned int)pending_hex_chars_expected_);
                        return;
                    }
                } else {
                    ESP_LOGE(TAG, "Missing content, args size: %u", (unsigned int)arguments.size());
                }

                std::lock_guard<std::mutex> lock(mutex_);
                body_.append(decoded_data);

                if (response_chunked_) {
                    eof_ = (arguments.size() >= 5) ? (arguments[4].int_value == 0) : false;
                } else {
                    eof_ = (arguments.size() >= 4) ? (arguments[3].int_value >= arguments[2].int_value) : false;
                }

                body_offset_ += decoded_data.size();
                if (arguments.size() >= 4 && (size_t)arguments[3].int_value > body_offset_) {
                    ESP_LOGW(TAG, "Possible stream gap: body_offset=%u, reported=%d",
                             (unsigned int)body_offset_, arguments[3].int_value);
                }
                cv_.notify_one();
            } else if (type == "err") {
                error_code_ = (arguments.size() >= 3) ? arguments[2].int_value : 255;
                xEventGroupSetBits(event_group_handle_, ML307_HTTP_EVENT_ERROR);
            } else if (type == "ind") {
                xEventGroupSetBits(event_group_handle_, ML307_HTTP_EVENT_IND);
            } else {
                ESP_LOGE(TAG, "Unknown HTTP event: %s", type.c_str());
            }
        }
    } else if (command == "MHTTPCREATE") {
        if (arguments.empty()) {
            ESP_LOGE(TAG, "Invalid MHTTPCREATE args");
            return;
        }
        http_id_ = arguments[0].int_value;
        instance_active_ = true;
        xEventGroupSetBits(event_group_handle_, ML307_HTTP_EVENT_INITIALIZED);
    } else if (command == "FIFO_OVERFLOW" ||
               (command == "SYSTEM_ERROR" && !arguments.empty() &&
                arguments[0].type == AtArgumentValue::Type::String &&
                arguments[0].string_value == "FIFO_OVERFLOW")) {
        error_code_ = 9;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            eof_ = true;
        }
        cv_.notify_one();
        xEventGroupSetBits(event_group_handle_, ML307_HTTP_EVENT_ERROR);
    }
}

void Ml307Http::SetHeader(const std::string& key, const std::string& value) {
    headers_[key] = value;
}

void Ml307Http::SetContent(std::string&& content) {
    content_ = std::make_optional(std::move(content));
}

void Ml307Http::SetTimeout(int timeout_ms) {
    timeout_ms_ = timeout_ms;
}

void Ml307Http::ParseResponseHeaders(const std::string& headers) {
    std::istringstream iss(headers);
    std::string line;
    while (std::getline(iss, line)) {
        std::istringstream line_iss(line);
        std::string key, value;
        std::getline(line_iss, key, ':');
        std::getline(line_iss, value);
    
        // 去除前后空格
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t\r\n") + 1);
        
        response_headers_[key] = value;

        // 检查是否为chunked传输编码
        if (key == "Transfer-Encoding" && value.find("chunked") != std::string::npos) {
            response_chunked_ = true;
            ESP_LOGI(TAG, "Found chunked transfer encoding");
        }
    }
}

bool Ml307Http::Open(const std::string& method, const std::string& url) {
    if (instance_active_) {
        Close();
    }
    xEventGroupClearBits(event_group_handle_,
                         ML307_HTTP_EVENT_INITIALIZED | ML307_HTTP_EVENT_ERROR |
                             ML307_HTTP_EVENT_HEADERS_RECEIVED | ML307_HTTP_EVENT_IND);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        body_.clear();
        body_offset_ = 0;
        eof_ = false;
    }
    status_code_ = -1;
    error_code_ = -1;
    content_length_ = 0;
    response_headers_.clear();
    response_chunked_ = false;
    request_chunked_ = false;
    pending_total_len_ = 0;
    pending_reported_offset_ = 0;
    pending_chunk_len_ = 0;
    pending_hex_chars_expected_ = 0;
    pending_hex_data_.clear();

    method_ = method;
    url_ = url;
    
    // 判断是否为需要发送内容的HTTP方法
    bool method_supports_content = (method_ == "POST" || method_ == "PUT");
    
    // 解析URL
    size_t protocol_end = url.find("://");
    if (protocol_end != std::string::npos) {
        protocol_ = url.substr(0, protocol_end);
        size_t host_start = protocol_end + 3;
        size_t path_start = url.find("/", host_start);
        if (path_start != std::string::npos) {
            host_ = url.substr(host_start, path_start - host_start);
            path_ = url.substr(path_start);
        } else {
            host_ = url.substr(host_start);
            path_ = "/";
        }
    } else {
        // URL格式不正确
        ESP_LOGE(TAG, "Invalid URL format");
        return false;
    }

    // 创建HTTP连接
    std::string command = "AT+MHTTPCREATE=\"" + protocol_ + "://" + host_ + "\"";
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to create HTTP connection");
        return false;
    }

    auto bits = xEventGroupWaitBits(event_group_handle_, ML307_HTTP_EVENT_INITIALIZED, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms_));
    if (!(bits & ML307_HTTP_EVENT_INITIALIZED)) {
        ESP_LOGE(TAG, "Timeout waiting for HTTP connection to be created");
        return false;
    }
    request_chunked_ = method_supports_content && !content_.has_value();
    ESP_LOGI(TAG, "HTTP connection created, ID: %d, protocol: %s, host: %s", http_id_, protocol_.c_str(), host_.c_str());

    if (protocol_ == "https") {
        command = "AT+MHTTPCFG=\"ssl\"," + std::to_string(http_id_) + ",1,0";
        at_uart_->SendCommand(command);
    }

    if (request_chunked_) {
        command = "AT+MHTTPCFG=\"chunked\"," + std::to_string(http_id_) + ",1";
        at_uart_->SendCommand(command);
    }

    // Set HEX encoding OFF
    command = "AT+MHTTPCFG=\"encoding\"," + std::to_string(http_id_) + ",0,0";
    at_uart_->SendCommand(command);

    // Set timeout (seconds): connect timeout, response timeout, input timeout
    // sprintf(command, "AT+MHTTPCFG=\"timeout\",%d,%d,%d,%d", http_id_, timeout_ms_ / 1000, timeout_ms_ / 1000, timeout_ms_ / 1000);
    // modem_.Command(command);

    // Set headers
    for (auto it = headers_.begin(); it != headers_.end(); it++) {
        auto line = it->first + ": " + it->second;
        bool is_last = std::next(it) == headers_.end();
        command = "AT+MHTTPHEADER=" + std::to_string(http_id_) + "," + std::to_string(is_last ? 0 : 1) + "," + std::to_string(line.size()) + ",\"" + line + "\"";
        at_uart_->SendCommand(command);
    }

    if (method_supports_content && content_.has_value()) {
        command = "AT+MHTTPCONTENT=" + std::to_string(http_id_) + ",0," + std::to_string(content_.value().size());
        at_uart_->SendCommand(command);
        at_uart_->SendCommand(content_.value());
        content_ = std::nullopt;
    }

    // Set HEX encoding ON
    command = "AT+MHTTPCFG=\"encoding\"," + std::to_string(http_id_) + ",1,1";
    at_uart_->SendCommand(command);

    // Send request
    // method to value: 1. GET 2. POST 3. PUT 4. DELETE 5. HEAD
    const char* methods[6] = {"UNKNOWN", "GET", "POST", "PUT", "DELETE", "HEAD"};
    int method_value = 1;
    for (int i = 0; i < 6; i++) {
        if (strcmp(methods[i], method_.c_str()) == 0) {
            method_value = i;
            break;
        }
    }
    command = "AT+MHTTPREQUEST=" + std::to_string(http_id_) + "," + std::to_string(method_value) + ",0,";
    if (!at_uart_->SendCommand(command + at_uart_->EncodeHex(path_))) {
        ESP_LOGE(TAG, "Failed to send HTTP request");
        return false;
    }

    if (request_chunked_) {
        auto bits = xEventGroupWaitBits(event_group_handle_, ML307_HTTP_EVENT_IND, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms_));
        if (!(bits & ML307_HTTP_EVENT_IND)) {
            ESP_LOGE(TAG, "Timeout waiting for HTTP IND");
            return false;
        }
    }
    return true;
}

bool Ml307Http::FetchHeaders() {
    // Wait for headers
    auto bits = xEventGroupWaitBits(event_group_handle_, ML307_HTTP_EVENT_HEADERS_RECEIVED | ML307_HTTP_EVENT_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms_));
    if (bits & ML307_HTTP_EVENT_ERROR) {
        ESP_LOGE(TAG, "HTTP request error: %s", ErrorCodeToString(error_code_).c_str());
        return false;
    }
    if (!(bits & ML307_HTTP_EVENT_HEADERS_RECEIVED)) {
        ESP_LOGE(TAG, "Timeout waiting for HTTP headers to be received");
        return false;
    }

    auto it = response_headers_.find("Content-Length");
    if (it != response_headers_.end()) {
        content_length_ = std::stoul(it->second);
    }

    ESP_LOGI(TAG, "HTTP request successful, status code: %d", status_code_);
    return true;
}

int Ml307Http::GetStatusCode() {
    if (status_code_ == -1) {
        if (!FetchHeaders()) {
            return -1;
        }
    }
    return status_code_;
}

size_t Ml307Http::GetBodyLength() {
    if (status_code_ == -1) {
        if (!FetchHeaders()) {
            return 0;
        }
    }
    return content_length_;
}

std::string Ml307Http::ReadAll() {
    std::unique_lock<std::mutex> lock(mutex_);
    
    auto timeout = std::chrono::milliseconds(timeout_ms_);
    bool received = cv_.wait_for(lock, timeout, [this] { 
        return eof_; 
    });
    
    if (!received) {
        ESP_LOGE(TAG, "Timeout waiting for HTTP content to be received");
        return body_;
    }
    
    return body_;
}

void Ml307Http::Close() {
    if (!instance_active_) {
        return;
    }
    std::string command = "AT+MHTTPDEL=" + std::to_string(http_id_);
    at_uart_->SendCommand(command);

    instance_active_ = false;
    eof_ = true;
    cv_.notify_one();
    ESP_LOGI(TAG, "HTTP connection closed, ID: %d", http_id_);
}

std::string Ml307Http::ErrorCodeToString(int error_code) {
    switch (error_code) {
        case 1: return "域名解析失败";
        case 2: return "连接服务器失败";
        case 3: return "连接服务器超时";
        case 4: return "SSL握手失败";
        case 5: return "连接异常断开";
        case 6: return "请求响应超时";
        case 7: return "接收数据解析失败";
        case 8: return "缓存空间不足";
        case 9: return "数据丢包";
        case 10: return "写文件失败";
        case 255: return "未知错误";
        default: return "未定义错误";
    }
}

std::string Ml307Http::GetResponseHeader(const std::string& key) const {
    auto it = response_headers_.find(key);
    if (it != response_headers_.end()) {
        return it->second;
    }
    return "";
}
