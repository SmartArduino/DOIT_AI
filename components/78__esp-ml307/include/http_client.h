#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include "http.h"
#include "tcp.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <map>
#include <string>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <memory>
#include <deque>
#include <cstring>
#include <atomic>

#define EC801E_HTTP_EVENT_HEADERS_RECEIVED (1 << 0)
#define EC801E_HTTP_EVENT_BODY_RECEIVED (1 << 1)
#define EC801E_HTTP_EVENT_ERROR (1 << 2)
#define EC801E_HTTP_EVENT_COMPLETE (1 << 3)

class NetworkInterface;

class HttpClient : public Http {
public:
    HttpClient(NetworkInterface* network, int connect_id = 0);
    ~HttpClient();

    void SetTimeout(int timeout_ms) override;
    void SetHeader(const std::string& key, const std::string& value) override;
    void SetContent(std::string&& content) override;
    bool Open(const std::string& method, const std::string& url) override;
    void Close() override;
    int Read(char* buffer, size_t buffer_size) override;
    int Write(const char* buffer, size_t buffer_size) override;

    int GetStatusCode() override;
    std::string GetResponseHeader(const std::string& key) const override;
    size_t GetBodyLength() override;
    std::string ReadAll() override;

private:
    // 数据块结构，用于队列缓冲
    struct DataChunk {
        std::string data;
        size_t offset = 0;  // 当前读取偏移
        
        // 使用移动构造函数避免拷贝
        DataChunk(std::string&& d) : data(std::move(d)), offset(0) {}
        DataChunk(const std::string& d) : data(d), offset(0) {}
        
        size_t available() const {
            return data.size() - offset;
        }
        
        size_t read(char* buffer, size_t size) {
            size_t bytes_to_read = std::min(size, available());
            if (bytes_to_read > 0) {
                memcpy(buffer, data.data() + offset, bytes_to_read);
                offset += bytes_to_read;
            }
            return bytes_to_read;
        }
        
        bool empty() const {
            return offset >= data.size();
        }
    };

    // 头部条目结构体，用于高效存储和查找
    struct HeaderEntry {
        std::string original_key;  // 保留原始大小写的key（用于输出HTTP头部）
        std::string value;         // 头部值
        
        HeaderEntry() = default;
        HeaderEntry(const std::string& key, const std::string& val) 
            : original_key(key), value(val) {}
    };

    NetworkInterface* network_;
    int connect_id_;
    std::unique_ptr<Tcp> tcp_;
    EventGroupHandle_t event_group_handle_;
    std::mutex mutex_;
    std::condition_variable cv_;
    
    // 用于读取操作的专门锁和缓冲区队列
    std::mutex read_mutex_;
    std::deque<DataChunk> body_chunks_;
    std::condition_variable write_cv_;
    const size_t MAX_BODY_CHUNKS_SIZE = 8192;
    std::mutex close_mutex_;
    std::atomic<bool> close_in_progress_{false};
    
    int status_code_ = -1;
    int timeout_ms_ = 30000;
    std::string rx_buffer_;
    std::map<std::string, HeaderEntry> headers_;  // key为小写，用于快速查找
    std::string url_;
    std::string method_;
    std::string protocol_;
    std::string host_;
    std::string path_;
    int port_ = 80;
    std::optional<std::string> content_ = std::nullopt;
    std::map<std::string, HeaderEntry> response_headers_;  // key为小写，用于快速查找
    
    // 移除原来的 body_ 变量，现在使用 body_chunks_ 队列
    size_t body_offset_ = 0;
    size_t content_length_ = 0;
    size_t total_body_received_ = 0;  // 总共接收的响应体字节数
    bool eof_ = false;
    bool connected_ = false;
    bool headers_received_ = false;
    bool request_chunked_ = false;
    bool response_chunked_ = false;
    bool connection_error_ = false;  // 新增：标记连接是否异常断开
    
    // HTTP 协议解析状态
    enum class ParseState {
        STATUS_LINE,
        HEADERS,
        BODY,
        CHUNK_SIZE,
        CHUNK_DATA,
        CHUNK_TRAILER,
        COMPLETE
    };
    ParseState parse_state_ = ParseState::STATUS_LINE;
    size_t chunk_size_ = 0;
    size_t chunk_received_ = 0;

    // 私有方法
    bool ParseUrl(const std::string& url);
    std::string BuildHttpRequest();
    void OnTcpData(const std::string& data);
    void OnTcpDisconnected();
    void ProcessReceivedData();
    bool ParseStatusLine(const std::string& line);
    bool ParseHeaderLine(const std::string& line);
    void ParseChunkedBody(const std::string& data);
    void ParseRegularBody(const std::string& data);
    size_t ParseChunkSize(const std::string& chunk_size_line);
    std::string GetNextLine(std::string& buffer);
    bool HasCompleteLine(const std::string& buffer);
    void SetError();
    
    // 新增：向读取队列添加数据的方法
    void AddBodyData(const std::string& data);
    void AddBodyData(std::string&& data);  // 移动版本
    
    // 新增：检查数据是否完整接收
    bool IsDataComplete() const;
};

#endif
