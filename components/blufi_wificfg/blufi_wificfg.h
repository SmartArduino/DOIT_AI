#ifndef __BLUFI_APP_H__
#define __BLUFI_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

#include "esp_wifi.h"

typedef struct {
    void (*sta_config_cb)(const wifi_config_t *wifi_config, void *arg);
    void (*custom_data_cb)(const uint8_t *data, size_t len, void *arg);
}blufi_wificfg_cbs_t;

esp_err_t blufi_wificfg_send_custom(uint8_t *data, size_t len);

esp_err_t blufi_wificfg_start(bool init_wifi, char *device_name, blufi_wificfg_cbs_t cbs, void *cbs_arg);

esp_err_t blufi_wificfg_stop(void);

#ifdef __cplusplus
}
#endif

#endif //__BLUFI_APP_H__