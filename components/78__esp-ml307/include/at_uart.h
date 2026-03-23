#ifndef _AT_UART_H_
#define _AT_UART_H_

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <list>
#include <cstdlib>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <driver/uart.h>

// UART事件定义
#define AT_EVENT_DATA_AVAILABLE BIT1
#define AT_EVENT_COMMAND_DONE   BIT2
#define AT_EVENT_COMMAND_ERROR  BIT3

// 默认配置
#define UART_NUM                UART_NUM_1

// 缓冲区大小配置
#define AT_UART_MAX_UNKNOWN_BUFFER_SIZE  16384  // 未知数据的最大缓冲区大小（16KB）
#define AT_UART_MAX_URC_BUFFER_SIZE      10240   // 已知 URC 的最大缓冲区大小（10KB），用于音频流等大数据
#define AT_UART_BUFFER_DUMP_SIZE         128     // 缓冲区转储时输出的字节数

// AT命令参数值结构
struct AtArgumentValue {
    enum class Type { String, Int, Double };
    Type type;
    std::string string_value;
    int int_value;
    double double_value;
    
    std::string ToString() const {
        switch (type) {
            case Type::String:
                return "\"" + string_value + "\"";
            case Type::Int:
                return std::to_string(int_value);
            case Type::Double:
                return std::to_string(double_value);
            default:
                return "";
        }
    }
};

// 数据接收回调函数类型
typedef std::function<void(const std::string& command, const std::vector<AtArgumentValue>& arguments)> UrcCallback;

class AtUart {
public:
    // 构造函数
    AtUart(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin = GPIO_NUM_NC);
    ~AtUart();

    // 初始化和配置
    void Initialize();
    
    // 波特率管理
    bool SetBaudRate(int new_baud_rate);
    int GetBaudRate() const { return baud_rate_; }
    
    // 数据发送
    bool SendCommand(const std::string& command, size_t timeout_ms = 1000, bool add_crlf = true);
    bool SendCommandWithData(const std::string& command, size_t timeout_ms = 1000, bool add_crlf = true, const char* data = nullptr, size_t data_length = 0);
    const std::string& GetResponse() const { return response_; }
    int GetCmeErrorCode() const { return cme_error_code_; }
    
    // 回调管理
    std::list<UrcCallback>::iterator RegisterUrcCallback(UrcCallback callback);
    void UnregisterUrcCallback(std::list<UrcCallback>::iterator iterator);
    
    // 控制接口
    void SetDtrPin(bool high);
    bool IsInitialized() const { return initialized_; }

    std::string EncodeHex(const std::string& data);
    std::string DecodeHex(const std::string& data);
    void EncodeHexAppend(std::string& dest, const char* data, size_t length);
    void DecodeHexAppend(std::string& dest, const char* data, size_t length);

private:
    // 配置参数
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t dtr_pin_;
    uart_port_t uart_num_;
    int baud_rate_;
    bool initialized_;
    int cme_error_code_ = 0;
    std::string response_;
    bool wait_for_response_ = false;
    std::mutex command_mutex_;
    std::mutex mutex_;
    
    // FreeRTOS 对象
    TaskHandle_t event_task_handle_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;
    QueueHandle_t event_queue_handle_;
    EventGroupHandle_t event_group_handle_;
    
    std::string rx_buffer_;
    
    // 回调函数
    std::list<UrcCallback> urc_callbacks_;
    
    // 内部方法
    void EventTask();
    void ReceiveTask();
    bool ParseResponse();
    bool DetectBaudRate();
    // 处理 AT 命令
    void HandleCommand(const char* command);
    // 处理 URC
    void HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments);
    bool SendData(const char* data, size_t length);

    // 静态任务函数
    static void EventTaskWrapper(void* arg);
};

#endif // _AT_UART_H_
