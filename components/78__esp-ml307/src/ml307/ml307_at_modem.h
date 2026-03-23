#ifndef _ML307_AT_MODEM_H_
#define _ML307_AT_MODEM_H_

#include "at_modem.h"
#include "tcp.h"
#include "udp.h"
#include "http.h"
#include "mqtt.h"
#include "web_socket.h"

class Ml307AtModem : public AtModem {
public:
    Ml307AtModem(std::shared_ptr<AtUart> at_uart);
    ~Ml307AtModem() override = default;

    void Reboot() override;
    bool SetSleepMode(bool enable, int delay_seconds=0) override;
    NetworkStatus WaitForNetworkReady(int timeout_ms=-1) override;

    // 实现基类的纯虚函数
    std::unique_ptr<Http> CreateHttp(int connect_id) override;
    std::unique_ptr<Tcp> CreateTcp(int connect_id) override;
    std::unique_ptr<Tcp> CreateSsl(int connect_id) override;
    std::unique_ptr<Udp> CreateUdp(int connect_id) override;
    std::unique_ptr<Mqtt> CreateMqtt(int connect_id) override;
    std::unique_ptr<WebSocket> CreateWebSocket(int connect_id) override;

protected:
    void HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) override;
    void ResetConnections();
};


#endif // _ML307_AT_MODEM_H_