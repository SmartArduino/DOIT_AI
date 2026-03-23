#ifndef ML307_SSL_H
#define ML307_SSL_H

#include "ml307_tcp.h"

class Ml307Ssl : public Ml307Tcp {
public:
    Ml307Ssl(std::shared_ptr<AtUart> at_uart, int tcp_id);
    ~Ml307Ssl() = default;

protected:
    // 重写SSL配置方法
    bool ConfigureSsl(int port) override;
};

#endif // ML307_SSL_H 