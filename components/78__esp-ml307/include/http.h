#ifndef HTTP_H
#define HTTP_H

#include <string>
#include <map>
#include <functional>

class Http {
public:
    virtual ~Http() = default;

    // Set timeout
    virtual void SetTimeout(int timeout_ms) = 0;

    // 设置 HTTP 请求头
    virtual void SetHeader(const std::string& key, const std::string& value) = 0;

    // 设置 HTTP Content
    virtual void SetContent(std::string&& content) = 0;

    // 打开 HTTP 连接并发送请求
    virtual bool Open(const std::string& method, const std::string& url) = 0;

    // 关闭 HTTP 连接
    virtual void Close() = 0;

    // 读取 HTTP 响应数据
    virtual int Read(char* buffer, size_t buffer_size) = 0;

    // 写入 HTTP 请求数据
    virtual int Write(const char* buffer, size_t buffer_size) = 0;

    // 获取 HTTP 响应状态码
    virtual int GetStatusCode() = 0;

    // 获取指定 key 的 HTTP 响应头
    virtual std::string GetResponseHeader(const std::string& key) const = 0;

    // 获取 HTTP 响应体长度
    virtual size_t GetBodyLength() = 0;

    // 获取 HTTP 响应体
    virtual std::string ReadAll() = 0;
};

#endif // HTTP_H