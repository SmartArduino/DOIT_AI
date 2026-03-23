#ifndef _AT_MODEM_H_
#define _AT_MODEM_H_

#include <cstddef>
#include <string>
#include <vector>
#include <list>
#include <functional>
#include <mutex>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include "at_uart.h"
#include "network_interface.h"

#define AT_EVENT_PIN_ERROR      BIT2
#define AT_EVENT_NETWORK_ERROR  BIT3
#define AT_EVENT_NETWORK_READY  BIT4

enum class NetworkStatus {
    ErrorInsertPin = -1,
    ErrorRegistrationDenied = -2,
    ErrorTimeout = -3,
    Ready = 0,
    Error = 1,
};

struct CeregState {
    int stat = 0;          // <stat>
    std::string tac;        // <tac>
    std::string ci;         // <ci>
    int AcT = -1;           // <AcT>

    std::string ToString() const {
        std::string json = "{";
        json += "\"stat\":" + std::to_string(stat);
        if (!tac.empty()) json += ",\"tac\":\"" + tac + "\"";
        if (!ci.empty()) json += ",\"ci\":\"" + ci + "\"";
        if (AcT >= 0) json += ",\"AcT\":" + std::to_string(AcT);
        json += "}";
        return json;
    }
};

class AtModem : public NetworkInterface {
public:
    // 静态检测方法
    static std::unique_ptr<AtModem> Detect(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin = GPIO_NUM_NC, int baud_rate = 115200);
    
    // 构造函数和析构函数
    AtModem(std::shared_ptr<AtUart> at_uart);
    virtual ~AtModem();
    std::shared_ptr<AtUart> GetAtUart() { return at_uart_; }
    void OnNetworkStateChanged(std::function<void(bool network_ready)> callback);

    // 网络状态管理
    virtual void Reboot();
    virtual NetworkStatus WaitForNetworkReady(int timeout_ms = -1);
    virtual bool SetSleepMode(bool enable, int delay_seconds=0);
    virtual void SetFlightMode(bool enable);

    // 模组信息获取
    std::string GetImei();
    std::string GetIccid();
    std::string GetModuleRevision();
    CeregState GetRegistrationState();
    std::string GetCarrierName();
    int GetCsq();

    // 状态查询
    bool pin_ready() const { return pin_ready_; }
    bool network_ready() const { return network_ready_; }

protected:
    std::shared_ptr<AtUart> at_uart_;
    std::mutex mutex_;
    std::string iccid_;
    std::string imei_;
    std::string carrier_name_;
    std::string module_revision_;
    int csq_ = -1;
    bool pin_ready_ = true;
    bool network_ready_ = false;

    gpio_num_t dtr_pin_;
    EventGroupHandle_t event_group_handle_ = nullptr;

    CeregState cereg_state_;

    virtual void HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments);

    std::function<void(bool network_state)> on_network_state_changed_;
};

#endif // _AT_MODEM_H_ 