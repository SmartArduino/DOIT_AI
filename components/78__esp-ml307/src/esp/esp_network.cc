#include "esp_network.h"

#include "esp_tcp.h"
#include "esp_ssl.h"
#include "esp_udp.h"
#include "esp_mqtt.h"
#include "http_client.h"
#include "web_socket.h"


EspNetwork::EspNetwork() {

}

EspNetwork::~EspNetwork() {

}

std::unique_ptr<Http> EspNetwork::CreateHttp(int connect_id) {
    return std::make_unique<HttpClient>(this, connect_id);
}

std::unique_ptr<Tcp> EspNetwork::CreateTcp(int connect_id) {
    return std::make_unique<EspTcp>();
}

std::unique_ptr<Tcp> EspNetwork::CreateSsl(int connect_id) {
    return std::make_unique<EspSsl>();
}

std::unique_ptr<Udp> EspNetwork::CreateUdp(int connect_id) {
    return std::make_unique<EspUdp>();
}

std::unique_ptr<Mqtt> EspNetwork::CreateMqtt(int connect_id) {
    return std::make_unique<EspMqtt>();
}

std::unique_ptr<WebSocket> EspNetwork::CreateWebSocket(int connect_id) {
    return std::make_unique<WebSocket>(this, connect_id);
}