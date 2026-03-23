#include "esp_udp.h"

#include <esp_log.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

static const char *TAG = "EspUdp";

EspUdp::EspUdp() : udp_fd_(-1) {
    event_group_ = xEventGroupCreate();
}

EspUdp::~EspUdp() {
    Disconnect();

    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }
}

bool EspUdp::Connect(const std::string& host, int port) {
    // 确保先断开已有连接
    if (connected_) {
        Disconnect();
    }

    struct sockaddr_in server_addr;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    // host is domain
    struct hostent *server = gethostbyname(host.c_str());
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to get host by name");
        return false;
    }
    memcpy(&server_addr.sin_addr, server->h_addr, server->h_length);

    udp_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return false;
    }

    int ret = connect(udp_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        close(udp_fd_);
        udp_fd_ = -1;
        return false;
    }

    connected_ = true;

    xEventGroupClearBits(event_group_, ESP_UDP_EVENT_RECEIVE_TASK_EXIT);
    xTaskCreate([](void* arg) {
        EspUdp* udp = (EspUdp*)arg;
        udp->ReceiveTask();
        xEventGroupSetBits(udp->event_group_, ESP_UDP_EVENT_RECEIVE_TASK_EXIT);
        vTaskDelete(NULL);
#ifdef CONFIG_IDF_TARGET_ESP32C2
    }, "udp_receive", 2048, this, 1, &receive_task_handle_);
#else
    }, "udp_receive", 4096, this, 1, &receive_task_handle_);
#endif
    return true;
}

void EspUdp::Disconnect() {
    connected_ = false;

    if (udp_fd_ != -1) {
        close(udp_fd_);
        udp_fd_ = -1;

        auto bits = xEventGroupWaitBits(event_group_, ESP_UDP_EVENT_RECEIVE_TASK_EXIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
        if (!(bits & ESP_UDP_EVENT_RECEIVE_TASK_EXIT)) {
            ESP_LOGE(TAG, "Failed to wait for receive task exit");
        }
    }
}

int EspUdp::Send(const std::string& data) {
    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    int ret = send(udp_fd_, data.data(), data.size(), 0);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Send failed: ret=%d, errno=%d", ret, errno);
    }
    return ret;
}

void EspUdp::ReceiveTask() {
    std::string data;
    while (connected_) {
        data.resize(1500);
        int ret = recv(udp_fd_, data.data(), data.size(), 0);
        if (ret <= 0) {
            connected_ = false;
            break;
        }
        
        if (message_callback_) {
            data.resize(ret);
            message_callback_(data);
        }
    }
}
