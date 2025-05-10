#include "gapRemote.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "gap_remote"
static uint32_t start_time = 0;
void GapRemote::ble_sync_callback(void)
{
    auto &remote = GapRemote::GetInstance();
    /* 获取设备地址类型 */
    int rc = ble_hs_id_infer_auto(0, &remote.ble_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "无法推断设备地址类型");
        return;
    }

    uint8_t addr[6];
    ble_hs_id_copy_addr(remote.ble_addr_type, addr, NULL);
    ESP_LOGI(TAG, "device gap mac: %02X:%02X:%02X:%02X:%02X:%02X",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    struct ble_gap_adv_params adv_params;
    // 设置广播参数
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0x20; // 默认最小间隔
    adv_params.itvl_max = 0x40; // 默认最大间隔
    adv_params.channel_map = 0x07; // 37信道
} 

int GapRemote::ble_gap_event(struct ble_gap_event *event, void *arg) {
    auto& remote = GapRemote::GetInstance();
    switch (event->type) {
        case BLE_GAP_EVENT_ADV_COMPLETE: // 广播完成事件
            ESP_LOGW(TAG, "broadcast finish, %ld", pdMS_TO_TICKS(xTaskGetTickCount()-start_time));
            remote.is_broadcast = false;
            break;
        default:
            break;
    }
    return 1;
}

int GapRemote::start_adv(uint8_t *adv_data, uint8_t len, uint32_t duration_ms, bool force){
    if (is_broadcast && force==false)
    {
        ESP_LOGW(TAG, "The last broadcast is not over yet");
        return -1;
    }else if (is_broadcast && force)
    {
        ble_gap_adv_stop();
        is_broadcast=false;
    }
    
    int rc = ble_gap_adv_set_data((unsigned char *)adv_data, len);
    if (rc != 0) {
        ESP_LOGE(TAG, "fail set adv data, err: %d", rc);
        return -2;
    }

    /* 广播参数 */
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_NON, // 不允许连接
        .disc_mode = BLE_GAP_DISC_MODE_GEN, // 一般可发现模式
        .itvl_min = 0x20,        // 最小间隔
        .itvl_max = 0x40 ,        // 最大间隔
        .channel_map = 0x07  ,               // 使用信道 37
    };
    
    rc = ble_gap_adv_start(ble_addr_type, NULL, duration_ms, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start fail, err: %d", rc);
        return -2;
    }
    start_time = xTaskGetTickCount();
    is_broadcast = true;
    return 1;
}

GapRemote::GapRemote() {
    /* 初始化 NimBLE 核心 */
    nimble_port_init();
    /* 配置 Host */
    ble_hs_cfg.sync_cb = ble_sync_callback;
    nimble_port_freertos_init([](void *param)
    { nimble_port_run(); });
    ESP_LOGI(TAG, "NimBLE init Finsh");
}