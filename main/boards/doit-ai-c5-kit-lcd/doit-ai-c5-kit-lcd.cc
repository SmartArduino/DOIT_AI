
#include "dual_network_board.h"
#include "audio/codecs/vb6824_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/spi_common.h>

#include "settings.h"

#include <wifi_station.h>
#include <wifi_configuration_ap.h>
#include <ssid_manager.h>
#include "assets/lang_config.h"

#include "led/circular_strip.h"
#include "power_save_timer.h"
#include "backlight.h"
#include "esp_lcd_panel_st7789_fix.h"

#define TAG "CustomBoard"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);


class CustomBoard : public DualNetworkBoard
{
private:
  Button boot_button_;
  VbAduioCodec audio_codec_;
  LcdDisplay *display_ = nullptr;
  PwmBacklight *backlight_ = nullptr;
  PowerSaveTimer *power_save_timer_ = nullptr;
  CircularStrip *led_ = nullptr;
  // PullUp4GPin pullup = PullUp4GPin();

  void InitializeButtons()
  {
    boot_button_.OnClick([this]()
                         {
      auto& app = Application::GetInstance();
      if (GetNetworkType() == NetworkType::WIFI) {
        if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
          // cast to WifiBoard
          auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
          wifi_board.ResetWifiConfiguration();
        }
      }
      // app.ToggleChatState(); 
      if(Application::GetInstance().GetDeviceState() != kDeviceStateListening){
            Application::GetInstance().WakeWordInvoke("你好小智");
        }
    });

    boot_button_.OnDoubleClick([this]()
    {
      auto &app = Application::GetInstance();
      if (app.GetDeviceState() == kDeviceStateStarting || app.GetDeviceState() == kDeviceStateWifiConfiguring)
      {
        SwitchNetworkType();
      }
#if (defined(CONFIG_VB6824_OTA_SUPPORT) && CONFIG_VB6824_OTA_SUPPORT == 1)
      if (esp_timer_get_time() > 20 * 1000 * 1000)
      {
        ESP_LOGI(TAG, "double click, do not enter OTA mode %ld", (uint32_t)esp_timer_get_time());
        return;
      }
      else if (WifiStation::GetInstance().IsConnected())
      {
        audio_codec_.OtaStart(0);
      }

#endif
    });

    boot_button_.OnPressRepeaDone([this](uint16_t count)
    {
      if (count >= 3 && GetNetworkType() == NetworkType::WIFI)
      {
        auto& app = Application::GetInstance();
          auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
          wifi_board.ResetWifiConfiguration();
      } 
    });
  }

  void InitializeSpi()
  {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
    buscfg.miso_io_num = GPIO_NUM_NC;
    buscfg.sclk_io_num = DISPLAY_CLK_PIN;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = 64;
    buscfg.flags = SPICOMMON_BUSFLAG_SLAVE;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
  }

  void InitializeLcdDisplay()
  {
    esp_lcd_panel_io_handle_t panel_io = nullptr;
    esp_lcd_panel_handle_t panel = nullptr;
    esp_lcd_panel_io_spi_config_t io_config = {};

    io_config.cs_gpio_num = DISPLAY_CS_PIN;
    io_config.dc_gpio_num = DISPLAY_DC_PIN;
    io_config.spi_mode = 3;
    io_config.pclk_hz = 80 * 1000 * 1000;
    io_config.trans_queue_depth = 5;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = DISPLAY_RST_PIN;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

    esp_lcd_panel_reset(panel);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_lcd_panel_reset(panel);

    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
    esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
    esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    // esp_lcd_panel_set_gap(panel, 24, 0);//横屏需要设置x方向偏移24

    // int angle = (int)GetBoardCfg()->GetFunValue(kFunValueDisplayAngle);
    // AngleMap angle_map_ = {(angle) % 360, (angle + 90) % 360,
    //                        (angle + 180) % 360, (angle + 270) % 360};

    display_ = new SpiLcdDisplay(panel_io, panel,
                                 DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                 {
                                     .text_font = &font_puhui_16_4,
                                     .icon_font = &font_awesome_16_4,
                                     .emoji_font = font_emoji_64_init(),
                                 });
  }

  void InitializeBacklight()
  {
    backlight_ = new PwmBacklight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
  }
  void InitializeLed()
  {
    xTaskCreate(
        [](void *arg)
        {
          auto led = (CircularStrip **)arg;
          gpio_set_pull_mode(RBG_DI_GPIO, GPIO_PULLDOWN_ONLY);
          gpio_set_direction(RBG_DI_GPIO, GPIO_MODE_OUTPUT);
          (*led) = new CircularStrip(RBG_DI_GPIO, 1);
          vTaskDelete(NULL);
        },
        "led_init", 1024 * 8, &led_, 1, NULL);
  }

public:
  CustomBoard() : DualNetworkBoard(ML307_TX_GPIO,
                                   ML307_RX_GPIO, GPIO_NUM_NC),
                  boot_button_(BOOT_BUTTON_GPIO),
                  audio_codec_(CODEC_TX_GPIO,CODEC_RX_GPIO)
  {
    gpio_set_level(POWER_KRRP_GPIO, 1); // 保持高电平
    InitializeLed();

    InitializeButtons();

    InitializeSpi();

    InitializeLcdDisplay();

    InitializeBacklight();
    GetBacklight()->RestoreBrightness();

    // McpTools::GetInstance()->McpToolsInit();

    audio_codec_.OnWakeUp([this](const std::string &command)
                           {
            if (command == std::string(vb6824_get_wakeup_word())){
                if(Application::GetInstance().GetDeviceState() != kDeviceStateListening){
                    Application::GetInstance().WakeWordInvoke("你好小智");
                }
            }else if (command == "开始配网"){
                       if (GetNetworkType() == NetworkType::WIFI) {
          auto &wifi_board = static_cast<WifiBoard &>(GetCurrentBoard());
          wifi_board.ResetWifiConfiguration();
        }
            } });
  }

  virtual AudioCodec *GetAudioCodec() override
  {
    return &audio_codec_;
  }

  virtual Display *GetDisplay() override
  {
    return display_;
  }

  virtual Backlight *GetBacklight() override
  {
    return backlight_;
  }

  // virtual void SetPowerSaveMode(bool enabled) override
  // {
  //     if (!enabled && power_save_timer_)
  //     {
  //         power_save_timer_->WakeUp();
  //     }
  //     DualNetworkBoard::SetPowerSaveMode(enabled);
  // }

  virtual Led *GetLed() override
  {
    return (Led *)led_;
  }
};
DECLARE_BOARD(CustomBoard);
