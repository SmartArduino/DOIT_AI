#ifndef UDP_H
#define UDP_H


#include <string>
#include <functional>

class Udp {
public:
    virtual ~Udp() = default;
    virtual bool Connect(const std::string& host, int port) = 0;
    virtual void Disconnect() = 0;
    virtual int Send(const std::string& data) = 0;

    virtual void OnMessage(std::function<void(const std::string& data)> callback) {
        message_callback_ = std::move(callback);
    }
    bool connected() const { return connected_; }

protected:
    std::function<void(const std::string& data)> message_callback_;
    bool connected_ = false;
};

#endif // UDP_H
