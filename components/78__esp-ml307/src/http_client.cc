#include "http_client.h"
#include "network_interface.h"
#include <esp_log.h>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cctype>

static const char *TAG = "HttpClient";

HttpClient::HttpClient(NetworkInterface* network, int connect_id) : network_(network), connect_id_(connect_id) {
    event_group_handle_ = xEventGroupCreate();
}

HttpClient::~HttpClient() {
    if (connected_) {
        Close();
    }
    vEventGroupDelete(event_group_handle_);
}

void HttpClient::SetTimeout(int timeout_ms) {
    timeout_ms_ = timeout_ms;
}

void HttpClient::SetHeader(const std::string& key, const std::string& value) {
    // 转换key为小写用于存储和查找，但保留原始key用于输出
    std::string lower_key = key;
    std::transform(lower_key.begin(), lower_key.end(), lower_key.begin(), ::tolower);
    headers_[lower_key] = HeaderEntry(key, value);
}

void HttpClient::SetContent(std::string&& content) {
    content_ = std::move(content);
}

bool HttpClient::ParseUrl(const std::string& url) {
    // 解析 URL: protocol://host:port/path
    size_t protocol_end = url.find("://");
    if (protocol_end == std::string::npos) {
        ESP_LOGE(TAG, "Invalid URL format: %s", url.c_str());
        return false;
    }

    protocol_ = url.substr(0, protocol_end);
    std::transform(protocol_.begin(), protocol_.end(), protocol_.begin(), ::tolower);

    size_t host_start = protocol_end + 3;
    size_t path_start = url.find("/", host_start);
    size_t port_start = url.find(":", host_start);

    // 默认端口
    if (protocol_ == "https") {
        port_ = 443;
    } else {
        port_ = 80;
    }

    if (path_start == std::string::npos) {
        path_ = "/";
        if (port_start != std::string::npos) {
            host_ = url.substr(host_start, port_start - host_start);
            std::string port_str = url.substr(port_start + 1);
            char* endptr;
            long port = strtol(port_str.c_str(), &endptr, 10);
            if (endptr != port_str.c_str() && *endptr == '\0' && port > 0 && port <= 65535) {
                port_ = static_cast<int>(port);
            } else {
                ESP_LOGE(TAG, "Invalid port: %s", port_str.c_str());
                return false;
            }
        } else {
            host_ = url.substr(host_start);
        }
    } else {
        path_ = url.substr(path_start);
        if (port_start != std::string::npos && port_start < path_start) {
            host_ = url.substr(host_start, port_start - host_start);
            std::string port_str = url.substr(port_start + 1, path_start - port_start - 1);
            char* endptr;
            long port = strtol(port_str.c_str(), &endptr, 10);
            if (endptr != port_str.c_str() && *endptr == '\0' && port > 0 && port <= 65535) {
                port_ = static_cast<int>(port);
            } else {
                ESP_LOGE(TAG, "Invalid port: %s", port_str.c_str());
                return false;
            }
        } else {
            host_ = url.substr(host_start, path_start - host_start);
        }
    }

    ESP_LOGD(TAG, "Parsed URL: protocol=%s, host=%s, port=%d, path=%s",
             protocol_.c_str(), host_.c_str(), port_, path_.c_str());
    return true;
}

std::string HttpClient::BuildHttpRequest() {
    std::ostringstream request;

    // 请求行
    request << method_ << " " << path_ << " HTTP/1.1\r\n";

    // Host 头
    request << "Host: " << host_;
    if ((protocol_ == "http" && port_ != 80) || (protocol_ == "https" && port_ != 443)) {
        request << ":" << port_;
    }
    request << "\r\n";

    // 用户自定义头部（使用原始key输出）
    for (const auto& [lower_key, header_entry] : headers_) {
        request << header_entry.original_key << ": " << header_entry.value << "\r\n";
    }

    // 内容相关头部（仅在用户未设置时添加）
    bool user_set_content_length = headers_.find("content-length") != headers_.end();
    bool user_set_transfer_encoding = headers_.find("transfer-encoding") != headers_.end();
    bool has_content = content_.has_value() && !content_->empty();
    if (has_content && !user_set_content_length) {
        request << "Content-Length: " << content_->size() << "\r\n";
    } else if ((method_ == "POST" || method_ == "PUT") && !user_set_content_length && !user_set_transfer_encoding) {
        if (request_chunked_) {
            request << "Transfer-Encoding: chunked\r\n";
        } else {
            request << "Content-Length: 0\r\n";
        }
    }

    // 连接控制（仅在用户未设置时添加）
    if (headers_.find("connection") == headers_.end()) {
        request << "Connection: close\r\n";
    }

    // 结束头部
    request << "\r\n";
    ESP_LOGD(TAG, "HTTP request headers:\n%s", request.str().c_str());

    // 请求体
    if (has_content) {
        request << *content_;
    }

    return request.str();
}

bool HttpClient::Open(const std::string& method, const std::string& url) {
    close_in_progress_.store(false, std::memory_order_release);
    method_ = method;
    url_ = url;

    // 重置状态
    status_code_ = -1;
    response_headers_.clear();
    {
        std::lock_guard<std::mutex> read_lock(read_mutex_);
        body_chunks_.clear();
    }
    body_offset_ = 0;
    content_length_ = 0;
    total_body_received_ = 0;
    eof_ = false;
    headers_received_ = false;
    response_chunked_ = false;
    connection_error_ = false;  // 重置连接错误状态
    parse_state_ = ParseState::STATUS_LINE;
    chunk_size_ = 0;
    chunk_received_ = 0;
    rx_buffer_.clear();

    xEventGroupClearBits(event_group_handle_,
                         EC801E_HTTP_EVENT_HEADERS_RECEIVED |
                         EC801E_HTTP_EVENT_BODY_RECEIVED |
                         EC801E_HTTP_EVENT_ERROR |
                         EC801E_HTTP_EVENT_COMPLETE);

    // 解析 URL
    if (!ParseUrl(url)) {
        return false;
    }

    // 建立 TCP 连接
    if (protocol_ == "https") {
        tcp_ = network_->CreateSsl(connect_id_);
    } else {
        tcp_ = network_->CreateTcp(connect_id_);
    }

    // 设置 TCP 数据接收回调
    tcp_->OnStream([this](const std::string& data) {
        OnTcpData(data);
    });

    // 设置 TCP 断开连接回调
    tcp_->OnDisconnected([this]() {
        OnTcpDisconnected();
    });
    if (!tcp_->Connect(host_, port_)) {
        ESP_LOGE(TAG, "TCP connection failed");
        return false;
    }

    connected_ = true;
    request_chunked_ = (method_ == "POST" || method_ == "PUT") && !content_.has_value();

    // 构建并发送 HTTP 请求
    std::string http_request = BuildHttpRequest();
    if (tcp_->Send(http_request) <= 0) {
        ESP_LOGE(TAG, "Send HTTP request failed");
        tcp_->Disconnect();
        connected_ = false;
        return false;
    }

    return true;
}

void HttpClient::Close() {
    bool expected = false;
    if (!close_in_progress_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(close_mutex_);
        if (connected_) {
            connected_ = false;
            write_cv_.notify_all();
            if (tcp_) {
                tcp_->Disconnect();
            }
            ESP_LOGD(TAG, "HTTP connection closed");
        }

        eof_ = true;
        cv_.notify_all();
    }
}

void HttpClient::OnTcpData(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查 body_chunks_ 大小，如果超过 8KB 且 heap 小于 32KB 则阻塞
    {
        std::unique_lock<std::mutex> read_lock(read_mutex_);
        write_cv_.wait(read_lock, [this, size=data.size()] {
            size_t total_size = size;
            for (const auto& chunk : body_chunks_) {
                total_size += chunk.data.size();
            }
            size_t free_heap = esp_get_free_heap_size();
            return total_size < MAX_BODY_CHUNKS_SIZE || !connected_ || free_heap >= 32768;
        });
    }

    rx_buffer_.append(data);
    ProcessReceivedData();
    cv_.notify_one();
}

void HttpClient::OnTcpDisconnected() {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;

    // 检查数据是否完整接收
    if (headers_received_ && !IsDataComplete()) {
        // 如果已接收头部但数据不完整，标记为连接错误
        connection_error_ = true;
        ESP_LOGE(TAG, "Connection closed prematurely, expected %u bytes but only received %u bytes",
                 content_length_, total_body_received_);
    } else {
        // 数据完整或还未开始接收响应体，正常结束
        eof_ = true;
    }

    cv_.notify_all();  // 通知所有等待的读取操作
}

void HttpClient::ProcessReceivedData() {
    while (!rx_buffer_.empty() && parse_state_ != ParseState::COMPLETE) {
        switch (parse_state_) {
            case ParseState::STATUS_LINE: {
                if (!HasCompleteLine(rx_buffer_)) return;  // 需要更多数据

                std::string line = GetNextLine(rx_buffer_);

                if (ParseStatusLine(line)) {
                    parse_state_ = ParseState::HEADERS;
                } else {
                    SetError();
                    return;
                }
                break;
            }

            case ParseState::HEADERS: {
                if (!HasCompleteLine(rx_buffer_)) return;  // 需要更多数据

                std::string line = GetNextLine(rx_buffer_);

                // 检查是否为空行（头部结束标记）
                // GetNextLine 已经移除了 \r，所以空行就是 empty()
                if (line.empty()) {
                    // 检查是否为 chunked 编码
                    auto it = response_headers_.find("transfer-encoding");
                    if (it != response_headers_.end() &&
                        it->second.value.find("chunked") != std::string::npos) {
                        response_chunked_ = true;
                        parse_state_ = ParseState::CHUNK_SIZE;
                    } else {
                        parse_state_ = ParseState::BODY;
                        auto cl_it = response_headers_.find("content-length");
                        if (cl_it != response_headers_.end()) {
                            char* endptr;
                            unsigned long length = strtoul(cl_it->second.value.c_str(), &endptr, 10);
                            if (endptr != cl_it->second.value.c_str() && *endptr == '\0') {
                                content_length_ = static_cast<size_t>(length);
                            } else {
                                ESP_LOGE(TAG, "Invalid Content-Length: %s", cl_it->second.value.c_str());
                                content_length_ = 0;
                            }
                        }
                    }
                    // 头部结束
                    headers_received_ = true;
                    xEventGroupSetBits(event_group_handle_, EC801E_HTTP_EVENT_HEADERS_RECEIVED);
                } else {
                    if (!ParseHeaderLine(line)) {
                        SetError();
                        return;
                    }
                }
                break;
            }

            case ParseState::BODY: {
                if (response_chunked_) {
                    ParseChunkedBody(rx_buffer_);
                } else {
                    ParseRegularBody(rx_buffer_);
                }
                break;
            }

            case ParseState::CHUNK_SIZE: {
                if (!HasCompleteLine(rx_buffer_)) return;  // 需要更多数据

                std::string line = GetNextLine(rx_buffer_);

                chunk_size_ = ParseChunkSize(line);
                chunk_received_ = 0;

                if (chunk_size_ == 0) {
                    parse_state_ = ParseState::CHUNK_TRAILER;
                } else {
                    parse_state_ = ParseState::CHUNK_DATA;
                }
                break;
            }

            case ParseState::CHUNK_DATA: {
                size_t available = std::min(rx_buffer_.size(), chunk_size_ - chunk_received_);
                if (available > 0) {
                    std::string chunk_data = rx_buffer_.substr(0, available);
                    AddBodyData(std::move(chunk_data));
                    total_body_received_ += available;
                    rx_buffer_.erase(0, available);
                    chunk_received_ += available;

                    if (chunk_received_ == chunk_size_) {
                        // 跳过 chunk 后的 CRLF
                        if (rx_buffer_.size() >= 2 && rx_buffer_.substr(0, 2) == "\r\n") {
                            rx_buffer_.erase(0, 2);
                        }
                        parse_state_ = ParseState::CHUNK_SIZE;
                    }
                }
                if (available == 0) return;  // 需要更多数据
                break;
            }

            case ParseState::CHUNK_TRAILER: {
                if (!HasCompleteLine(rx_buffer_)) return;  // 需要更多数据

                std::string line = GetNextLine(rx_buffer_);

                if (line.empty()) {
                    parse_state_ = ParseState::COMPLETE;
                    eof_ = true;
                    xEventGroupSetBits(event_group_handle_, EC801E_HTTP_EVENT_COMPLETE);
                }
                // 忽略 trailer 头部
                break;
            }

            case ParseState::COMPLETE:
                return;
        }
    }

    // 检查是否完成（非 chunked 模式）
    if (parse_state_ == ParseState::BODY && !response_chunked_ &&
        content_length_ > 0 && total_body_received_ >= content_length_) {
        parse_state_ = ParseState::COMPLETE;
        eof_ = true;
        xEventGroupSetBits(event_group_handle_, EC801E_HTTP_EVENT_COMPLETE);
        ESP_LOGD(TAG, "HTTP response body received: %u/%u bytes", total_body_received_, content_length_);
    }
}

bool HttpClient::ParseStatusLine(const std::string& line) {
    // HTTP/1.1 200 OK
    std::istringstream iss(line);
    std::string version, status_str, reason;

    if (!(iss >> version >> status_str)) {
        ESP_LOGE(TAG, "Invalid status line: %s", line.c_str());
        return false;
    }

    std::getline(iss, reason);  // 获取剩余部分作为原因短语

    // 安全地解析状态码
    char* endptr;
    long status = strtol(status_str.c_str(), &endptr, 10);

    if (endptr == status_str.c_str() || *endptr != '\0' || status < 100 || status > 999) {
        ESP_LOGE(TAG, "Parse status code failed: %s", status_str.c_str());
        return false;
    }

    status_code_ = static_cast<int>(status);
    ESP_LOGD(TAG, "HTTP status code: %d", status_code_);
    return true;
}

bool HttpClient::ParseHeaderLine(const std::string& line) {
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
        ESP_LOGE(TAG, "Invalid header line: %s", line.c_str());
        return false;
    }

    std::string key = line.substr(0, colon_pos);
    std::string value = line.substr(colon_pos + 1);

    // 去除前后空格
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t\r\n") + 1);

    // 转换为小写键名用于存储和查找，同时保存原始key
    std::string lower_key = key;
    std::transform(lower_key.begin(), lower_key.end(), lower_key.begin(), ::tolower);

    response_headers_[lower_key] = HeaderEntry(key, value);

    return true;
}

void HttpClient::ParseChunkedBody(const std::string& data) {
    // Chunked body 在 ProcessReceivedData 中的状态机中处理
}

void HttpClient::ParseRegularBody(const std::string& data) {
    if (!data.empty()) {
        AddBodyData(data);  // 使用新的方法添加数据
        total_body_received_ += data.size();  // 累加接收的字节数
        rx_buffer_.clear();
    }
}

size_t HttpClient::ParseChunkSize(const std::string& chunk_size_line) {
    // 解析 chunk size（十六进制）
    std::string size_str = chunk_size_line;

    // 移除 CRLF 和任何扩展
    size_t semi_pos = size_str.find(';');
    if (semi_pos != std::string::npos) {
        size_str = size_str.substr(0, semi_pos);
    }

    size_str.erase(size_str.find_last_not_of(" \t\r\n") + 1);

    // 安全地解析十六进制 chunk 大小
    char* endptr;
    unsigned long chunk_size = strtoul(size_str.c_str(), &endptr, 16);

    if (endptr == size_str.c_str() || *endptr != '\0') {
        ESP_LOGE(TAG, "Parse chunk size failed: %s", size_str.c_str());
        return 0;
    }

    return static_cast<size_t>(chunk_size);
}

std::string HttpClient::GetNextLine(std::string& buffer) {
    size_t pos = buffer.find('\n');
    if (pos == std::string::npos) {
        return "";  // 没有完整的行
    }

    std::string line = buffer.substr(0, pos);
    buffer.erase(0, pos + 1);

    // 移除 CR
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    return line;
}

bool HttpClient::HasCompleteLine(const std::string& buffer) {
    return buffer.find('\n') != std::string::npos;
}

void HttpClient::SetError() {
    ESP_LOGE(TAG, "HTTP parse error");
    xEventGroupSetBits(event_group_handle_, EC801E_HTTP_EVENT_ERROR);
}

int HttpClient::Read(char* buffer, size_t buffer_size) {
    std::unique_lock<std::mutex> read_lock(read_mutex_);

    // 如果连接异常断开，返回错误
    if (connection_error_) {
        return -1;
    }

    // 如果已经到达文件末尾且没有更多数据，返回0
    if (eof_ && body_chunks_.empty()) {
        return 0;
    }

    // 如果有数据可读，直接返回
    while (!body_chunks_.empty()) {
        auto& front_chunk = body_chunks_.front();
        size_t bytes_read = front_chunk.read(buffer, buffer_size);

        if (bytes_read > 0) {
            // 如果当前chunk已读完，移除它
            if (front_chunk.empty()) {
                body_chunks_.pop_front();
            }
            // 通知等待的写入操作
            write_cv_.notify_one();
            return static_cast<int>(bytes_read);
        }

        // 如果chunk为空，移除它并继续下一个
        body_chunks_.pop_front();
    }

    // 如果连接已断开，检查是否有错误
    if (!connected_) {
        if (connection_error_) {
            return -1;  // 连接异常断开
        }
        return 0;  // 正常结束
    }

    // 等待数据或连接关闭
    auto timeout = std::chrono::milliseconds(timeout_ms_);
    bool received = cv_.wait_for(read_lock, timeout, [this] {
        return !body_chunks_.empty() || eof_ || !connected_ || connection_error_;
    });

    if (!received) {
        ESP_LOGE(TAG, "Wait for HTTP content receive timeout");
        return -1;
    }

    // 再次检查连接错误状态
    if (connection_error_) {
        return -1;
    }

    // 再次检查是否有数据可读
    while (!body_chunks_.empty()) {
        auto& front_chunk = body_chunks_.front();
        size_t bytes_read = front_chunk.read(buffer, buffer_size);

        if (bytes_read > 0) {
            // 如果当前chunk已读完，移除它
            if (front_chunk.empty()) {
                body_chunks_.pop_front();
            }
            // 通知等待的写入操作
            write_cv_.notify_one();
            return static_cast<int>(bytes_read);
        }

        // 如果chunk为空，移除它并继续下一个
        body_chunks_.pop_front();
    }

    // 连接已关闭或到达EOF，返回0
    return 0;
}

int HttpClient::Write(const char* buffer, size_t buffer_size) {
    if (!connected_ || !request_chunked_) {
        ESP_LOGE(TAG, "Cannot write: connection closed or not chunked mode");
        return -1;
    }

    if (buffer_size == 0) {
        // 发送结束 chunk
        std::string end_chunk = "0\r\n\r\n";
        return tcp_->Send(end_chunk);
    }

    // 发送 chunk
    std::ostringstream chunk;
    chunk << std::hex << buffer_size << "\r\n";
    chunk.write(buffer, buffer_size);
    chunk << "\r\n";

    std::string chunk_data = chunk.str();
    return tcp_->Send(chunk_data);
}

int HttpClient::GetStatusCode() {
    if (!headers_received_) {
        // 等待头部接收
        auto bits = xEventGroupWaitBits(event_group_handle_,
                                        EC801E_HTTP_EVENT_HEADERS_RECEIVED | EC801E_HTTP_EVENT_ERROR,
                                        pdFALSE, pdFALSE,
                                        pdMS_TO_TICKS(timeout_ms_));

        if (bits & EC801E_HTTP_EVENT_ERROR) {
            return -1;
        }
        if (!(bits & EC801E_HTTP_EVENT_HEADERS_RECEIVED)) {
            ESP_LOGE(TAG, "Wait for HTTP headers receive timeout");
            return -1;
        }
    }

    return status_code_;
}

std::string HttpClient::GetResponseHeader(const std::string& key) const {
    // 转换为小写进行查找
    std::string lower_key = key;
    std::transform(lower_key.begin(), lower_key.end(), lower_key.begin(), ::tolower);
    
    auto it = response_headers_.find(lower_key);
    if (it != response_headers_.end()) {
        return it->second.value;
    }
    return "";
}

size_t HttpClient::GetBodyLength() {
    if (!headers_received_) {
        GetStatusCode();  // 这会等待头部接收
    }

    if (response_chunked_) {
        return 0;  // Chunked 模式下长度未知
    }

    return content_length_;
}

void HttpClient::AddBodyData(const std::string& data) {
    if (data.empty()) return;

    std::lock_guard<std::mutex> read_lock(read_mutex_);
    body_chunks_.emplace_back(data);  // 使用构造函数，避免额外的拷贝
    cv_.notify_one();  // 通知有新数据
    write_cv_.notify_one();  // 通知写入操作
}

void HttpClient::AddBodyData(std::string&& data) {
    if (data.empty()) return;

    std::lock_guard<std::mutex> read_lock(read_mutex_);
    body_chunks_.emplace_back(std::move(data));  // 使用移动语义，避免拷贝
    cv_.notify_one();  // 通知有新数据
    write_cv_.notify_one();  // 通知写入操作
}

std::string HttpClient::ReadAll() {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待完成或出错
    auto timeout = std::chrono::milliseconds(timeout_ms_);
    bool completed = cv_.wait_for(lock, timeout, [this] {
        return eof_ || connection_error_;
    });

    if (!completed) {
        ESP_LOGE(TAG, "Wait for HTTP content receive complete timeout");
        return "";  // 超时返回空字符串
    }

    // 如果连接异常断开，返回空字符串并记录错误
    if (connection_error_) {
        ESP_LOGE(TAG, "Cannot read all data: connection closed prematurely");
        return "";
    }

    // 收集所有数据
    std::string result;
    std::lock_guard<std::mutex> read_lock(read_mutex_);
    for (const auto& chunk : body_chunks_) {
        result.append(chunk.data);
    }

    return result;
}

bool HttpClient::IsDataComplete() const {
    // 对于chunked编码，如果parse_state_是COMPLETE，说明接收完整
    if (response_chunked_) {
        return parse_state_ == ParseState::COMPLETE;
    }

    // 对于固定长度，检查是否接收了完整的content-length
    if (content_length_ > 0) {
        return total_body_received_ >= content_length_;
    }

    // 如果没有content-length且不是chunked，当连接关闭时认为完整
    // 这种情况通常用于HTTP/1.0或者content-length为0的响应
    return true;
}
