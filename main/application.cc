#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "opus_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"
#include "assets/zh/binary.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"


static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "fatal_error",
    "invalid_state"
};

Application::Application() {
    event_group_ = xEventGroupCreate();
#if (defined(CONFIG_OPUS_CODEC_TYPE_NO_CODEC))
    background_task_ = new BackgroundTask(2048);
#elif (defined(CONFIG_OPUS_CODEC_TYPE_ONLY_DECODE))
    background_task_ = new BackgroundTask(4096 * 2 + 512);
#else
    background_task_ = new BackgroundTask(4096 * 8);
#endif
}

Application::~Application() {
    if (background_task_ != nullptr) {
        delete background_task_;
    }
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    // Check if there is a new firmware version available
    ota_.SetPostData(board.GetJson());

    const int MAX_RETRY = 10;
    int retry_count = 0;

    while (true) {
        if (!ota_.CheckVersion()) {
            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "版本检查失败次数过多，退出检查");
                return;
            }
            ESP_LOGW(TAG, "版本检查失败，%d秒后重试 (%d/%d)", 60, retry_count, MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }
        retry_count = 0;

        if (ota_.HasNewVersion()) {
            Alert("OTA 升级", "正在升级系统", "happy", std::string_view(p3_upgrade_start, p3_upgrade_end - p3_upgrade_start));
            // Wait for the chat state to be idle
            do {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } while (GetDeviceState() != kDeviceStateIdle);

            // Use main task to do the upgrade, not cancelable
            Schedule([this, display]() {
                SetDeviceState(kDeviceStateUpgrading);
                
                display->SetIcon(FONT_AWESOME_DOWNLOAD);
                display->SetChatMessage("system", "新版本 " + ota_.GetFirmwareVersion());

                auto& board = Board::GetInstance();
                board.SetPowerSaveMode(false);
#if CONFIG_USE_AUDIO_PROCESSING
                wake_word_detect_.StopDetection();
#endif
                // 预先关闭音频输出，避免升级过程有音频操作
                auto codec = board.GetAudioCodec();
                codec->EnableInput(false);
                codec->EnableOutput(false);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    audio_decode_queue_.clear();
                }
                background_task_->WaitForCompletion();
                delete background_task_;
                background_task_ = nullptr;
                vTaskDelay(pdMS_TO_TICKS(1000));

                ota_.StartUpgrade([display](int progress, size_t speed) {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "%d%% %zuKB/s", progress, speed / 1024);
                    display->SetChatMessage("system", buffer);
                });

                // If upgrade success, the device will reboot and never reach here
                display->SetStatus("更新失败");
                ESP_LOGI(TAG, "Firmware upgrade failed...");
                vTaskDelay(pdMS_TO_TICKS(3000));
                Reboot();
            });

            return;
        }

        // No new version, mark the current version as valid
        ota_.MarkCurrentVersionValid();
        display->ShowNotification("版本 " + ota_.GetCurrentVersion());
    
        if (ota_.HasActivationCode()) {
            // Activation code is valid
            SetDeviceState(kDeviceStateActivating);
            ShowActivationCode();
            vTaskDelete(NULL);
            // Check again in 60 seconds
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        SetDeviceState(kDeviceStateIdle);
        display->SetChatMessage("system", "");

        // Exit the loop if upgrade or idle
        break;
    }
}

void Application::ShowActivationCode() {
    auto& message = ota_.GetActivationMessage();
    auto& code = ota_.GetActivationCode();

    struct digit_sound {
        char digit;
        const char* data;
        size_t size;
    };
    static const std::array<digit_sound, 10> digit_sounds{{
        digit_sound{'0', p3_0_start, size_t(p3_0_end - p3_0_start)},
        digit_sound{'1', p3_1_start, size_t(p3_1_end - p3_1_start)}, 
        digit_sound{'2', p3_2_start, size_t(p3_2_end - p3_2_start)},
        digit_sound{'3', p3_3_start, size_t(p3_3_end - p3_3_start)},
        digit_sound{'4', p3_4_start, size_t(p3_4_end - p3_4_start)},
        digit_sound{'5', p3_5_start, size_t(p3_5_end - p3_5_start)},
        digit_sound{'6', p3_6_start, size_t(p3_6_end - p3_6_start)},
        digit_sound{'7', p3_7_start, size_t(p3_7_end - p3_7_start)},
        digit_sound{'8', p3_8_start, size_t(p3_8_end - p3_8_start)},
        digit_sound{'9', p3_9_start, size_t(p3_9_end - p3_9_start)}
    }};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert("激活设备", message, "happy", std::string_view(p3_activation_start, p3_activation_end - p3_activation_start));
    vTaskDelay(pdMS_TO_TICKS(1000));
    background_task_->WaitForCompletion();

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
            [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            PlayLocalFile(it->data, it->size);
        }
    }
}

void Application::Alert(const std::string& status, const std::string& message, const std::string& emotion, const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert %s: %s [%s]", status.c_str(), message.c_str(), emotion.c_str());
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        PlayLocalFile(sound.data(), sound.size());
    }
}

void Application::PlayLocalFile(const char* data, size_t size) {
    ESP_LOGI(TAG, "PlayLocalFile: %zu bytes", size);
    auto codec = Board::GetInstance().GetAudioCodec();
    codec->EnableOutput(true);
    SetDecodeSampleRate(16000);
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        std::vector<uint8_t> opus;
        opus.resize(payload_size);
        memcpy(opus.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(opus));
    }
}

void Application::ToggleChatState() {
    Schedule([this]() {
        if (device_state_ == kDeviceStateActivating) {
            Reboot();
            return;
        }

        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }

        if (device_state_ == kDeviceStateIdle) {
            SetDeviceState(kDeviceStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                Alert("ERROR", "无法建立音频通道", "sad");
                SetDeviceState(kDeviceStateIdle);
                return;
            }

            keep_listening_ = true;
            protocol_->SendStartListening(kListeningModeAutoStop);
            SetDeviceState(kDeviceStateListening);
        } else if (device_state_ == kDeviceStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
        } else if (device_state_ == kDeviceStateListening) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::StartListening() {
    Schedule([this]() {
        if (device_state_ == kDeviceStateActivating) {
            Reboot();
            return;
        }

        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }
        
        keep_listening_ = false;
        if (device_state_ == kDeviceStateIdle) {
            if (!protocol_->IsAudioChannelOpened()) {
                SetDeviceState(kDeviceStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    SetDeviceState(kDeviceStateIdle);
                    Alert("ERROR", "无法建立音频通道", "sad");
                    return;
                }
            }
            protocol_->SendStartListening(kListeningModeManualStop);
            SetDeviceState(kDeviceStateListening);
        } else if (device_state_ == kDeviceStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
            protocol_->SendStartListening(kListeningModeManualStop);
            // FIXME: Wait for the speaker to empty the buffer
            vTaskDelay(pdMS_TO_TICKS(120));
            SetDeviceState(kDeviceStateListening);
        }
    });
}

void Application::StopListening() {
    Schedule([this]() {
        if (device_state_ == kDeviceStateListening) {
            protocol_->SendStopListening();
            SetDeviceState(kDeviceStateIdle);
        }
    });
}

void Application::Start() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();

    auto opus_codec = board.GetOpusCodec();
    opus_decode_sample_rate_ = codec->output_sample_rate();
    opus_codec->DecodeConfig(opus_decode_sample_rate_, 1, OPUS_FRAME_DURATION_MS);
    opus_codec->EncodeConfig(16000, 1, OPUS_FRAME_DURATION_MS);

#if CONFIG_USE_AUDIO_PROCESSING
    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
#endif

    codec->OnInputReady([this, codec]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if(xPortInIsrContext()){
            xEventGroupSetBitsFromISR(event_group_, AUDIO_INPUT_READY_EVENT, &higher_priority_task_woken);
        }else{
            xEventGroupSetBits(event_group_, AUDIO_INPUT_READY_EVENT); 
        }
        return higher_priority_task_woken == pdTRUE;
    });
    codec->OnOutputReady([this]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if(xPortInIsrContext()){
            xEventGroupSetBitsFromISR(event_group_, AUDIO_OUTPUT_READY_EVENT, &higher_priority_task_woken);
        }else{
            xEventGroupSetBits(event_group_, AUDIO_OUTPUT_READY_EVENT);  
        }
        return higher_priority_task_woken == pdTRUE;
    });
    codec->Start();

    /* Start the main loop */
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->MainLoop();
        vTaskDelete(NULL);
#ifdef CONFIG_IDF_TARGET_ESP32C2
    }, "main_loop", 4096, this, 2, nullptr);
#else
    }, "main_loop", 4096 * 2, this, 2, nullptr);
#endif
    /* Wait for the network to be ready */
    board.StartNetwork();

    // Initialize the protocol
    display->SetStatus("加载协议...");
#ifdef CONFIG_CONNECTION_TYPE_WEBSOCKET
    protocol_ = std::make_unique<WebsocketProtocol>();
#else
    protocol_ = std::make_unique<MqttProtocol>();
#endif
    protocol_->OnNetworkError([this](const std::string& message) {
        Alert("ERROR", message, "sad");
    });
    protocol_->OnIncomingAudio([this](std::vector<uint8_t>&& data) {
        if(device_state_ == kDeviceStateSpeaking && audio_decode_queue_.size() > 15){
#ifdef CONGIF_OPUS_CODEC_TYPE_NO_CODEC
            vTaskDelay(20 / portTICK_PERIOD_MS);
#else
            vTaskDelay(60 / portTICK_PERIOD_MS);
#endif
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (device_state_ == kDeviceStateSpeaking) {
            audio_decode_queue_.emplace_back(std::move(data));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "服务器的音频采样率 %d 与设备输出的采样率 %d 不一致，重采样后可能会失真",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
        SetDecodeSampleRate(protocol_->server_sample_rate());
        // 物联网设备描述符
        last_iot_states_.clear();
        auto& thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (device_state_ == kDeviceStateIdle || device_state_ == kDeviceStateListening) {
                        SetDeviceState(kDeviceStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (device_state_ == kDeviceStateSpeaking) {
                        background_task_->WaitForCompletion();
                        if (keep_listening_) {
                            protocol_->SendStartListening(kListeningModeAutoStop);
                            SetDeviceState(kDeviceStateListening);
                        } else {
                            SetDeviceState(kDeviceStateIdle);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (text != NULL) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([this, display, message = std::string(text->valuestring)]() {
                        display->SetChatMessage("assistant", message);
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (text != NULL) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([this, display, message = std::string(text->valuestring)]() {
                    display->SetChatMessage("user", message);
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (emotion != NULL) {
                Schedule([this, display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str);
                });
            }
        } else if (strcmp(type->valuestring, "iot") == 0) {
            auto commands = cJSON_GetObjectItem(root, "commands");
            if (commands != NULL) {
                auto& thing_manager = iot::ThingManager::GetInstance();
                for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
                    auto command = cJSON_GetArrayItem(commands, i);
                    thing_manager.Invoke(command);
                }
            }
        }
    });
    protocol_->Start();

    // Check for new firmware version or get the MQTT broker address
    ota_.SetCheckVersionUrl(CONFIG_OTA_VERSION_URL);
    ota_.SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    ota_.SetHeader("Client-Id", board.GetUuid());
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->CheckNewVersion();
        vTaskDelete(NULL);
#ifdef CONFIG_IDF_TARGET_ESP32C2
    }, "check_new_version", 4096, this, 1, nullptr);
#else
    }, "check_new_version", 4096 * 2, this, 1, nullptr);
#endif


#if CONFIG_USE_AUDIO_PROCESSING
    audio_processor_.Initialize(codec->input_channels(), codec->input_reference());
    audio_processor_.OnOutput([this](std::vector<int16_t>&& data) {
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            auto opus_codec = board.GetOpusCodec();
            opus_codec->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    });

    wake_word_detect_.Initialize(codec->input_channels(), codec->input_reference());
    wake_word_detect_.OnVadStateChange([this](bool speaking) {
        Schedule([this, speaking]() {
            if (device_state_ == kDeviceStateListening) {
                if (speaking) {
                    voice_detected_ = true;
                } else {
                    voice_detected_ = false;
                }
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        });
    });

    wake_word_detect_.OnWakeWordDetected([this](const std::string& wake_word) {
        Schedule([this, &wake_word]() {
            if (device_state_ == kDeviceStateIdle) {
                SetDeviceState(kDeviceStateConnecting);
                wake_word_detect_.EncodeWakeWordData();

                if (!protocol_->OpenAudioChannel()) {
                    ESP_LOGE(TAG, "Failed to open audio channel");
                    SetDeviceState(kDeviceStateIdle);
                    wake_word_detect_.StartDetection();
                    return;
                }
                
                std::vector<uint8_t> opus;
                // Encode and send the wake word data to the server
                while (wake_word_detect_.GetWakeWordOpus(opus)) {
                    protocol_->SendAudio(opus);
                }
                // Set the chat state to wake word detected
                protocol_->SendWakeWordDetected(wake_word);
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                keep_listening_ = true;
                SetDeviceState(kDeviceStateListening);
            } else if (device_state_ == kDeviceStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            }

            // Resume detection
            wake_word_detect_.StartDetection();
        });
    });
    wake_word_detect_.StartDetection();
#endif

    SetDeviceState(kDeviceStateIdle);
}

void Application::Schedule(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// The Main Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainLoop() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_,
            SCHEDULE_EVENT | AUDIO_INPUT_READY_EVENT | AUDIO_OUTPUT_READY_EVENT,
            pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & AUDIO_INPUT_READY_EVENT) {
            InputAudio();
        }
        if (bits & AUDIO_OUTPUT_READY_EVENT) {
            OutputAudio();
        }
        if (bits & SCHEDULE_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void Application::ResetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto opus_codec = Board::GetInstance().GetOpusCodec();
    opus_codec->DecodeResetState();
    audio_decode_queue_.clear();
    last_output_time_ = std::chrono::steady_clock::now();
}

void Application::OutputAudio() {
    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    while(1){
        lock.lock();
        if (audio_decode_queue_.empty()) {
            // Disable the output if there is no audio data for a long time
            if (device_state_ == kDeviceStateIdle) {
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
                if (duration > max_silence_seconds) {
                    codec->EnableOutput(false);
                }
            }
            return;
        }

        if (device_state_ == kDeviceStateListening) {
            audio_decode_queue_.clear();
            return;
        }

        last_output_time_ = now;
        auto opus = std::move(audio_decode_queue_.front());
        audio_decode_queue_.pop_front();
        lock.unlock();

        background_task_->Schedule([this, codec, opus = std::move(opus)]() mutable {
            if (aborted_) {
                return;
            }
            int resample = -1;
            std::vector<int16_t> pcm; 
            auto opus_codec = Board::GetInstance().GetOpusCodec();
            if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
                resample = codec->output_sample_rate();
            }
            if (!opus_codec->Decode(std::move(opus), pcm, resample)) {
                return;
            }
            codec->OutputData(pcm);
        });
    }
}

void Application::InputAudio() {
    auto codec = Board::GetInstance().GetAudioCodec();
    std::vector<int16_t> data;
    std::vector<int16_t> resampled_data;
    if (!codec->InputData(data)) {
        return;
    }
    
#if CONFIG_USE_AUDIO_PROCESSING
    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            resampled_data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                resampled_data[j] = resampled_mic[i];
                resampled_data[j + 1] = resampled_reference[i];
            }
        } else {
            resampled_data.resize(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled_data.data());
        }
    }
    
    if (audio_processor_.IsRunning()) {
        audio_processor_.Input(resampled_data);
    }
    if (wake_word_detect_.IsDetectionRunning()) {
        wake_word_detect_.Feed(resampled_data);
    }
#else
    if (device_state_ == kDeviceStateListening) {
        background_task_->Schedule([this, data = std::move(data)]() mutable {
            int resample = -1;
            auto codec = Board::GetInstance().GetAudioCodec();
            auto opus_codec = Board::GetInstance().GetOpusCodec();

            if(codec->input_sample_rate() != 16000){
                resample = 16000;
            }
            opus_codec->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            }, resample);
        });
    }
#endif
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    
    device_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[device_state_]);
    // The state is changed, wait for all background tasks to finish
    background_task_->WaitForCompletion();

    auto& board = Board::GetInstance();
    auto codec = board.GetAudioCodec();
    auto opus_codec = board.GetOpusCodec();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            display->SetStatus("待命");
            display->SetEmotion("neutral");
#ifdef CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Stop();
#endif
            break;
        case kDeviceStateConnecting:
            display->SetStatus("连接中...");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
            display->SetStatus("聆听中...");
            display->SetEmotion("neutral");
            ResetDecoder();
            opus_codec->EncodeResetState();
#if CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Start();
#endif
            UpdateIotStates();
            break;
        case kDeviceStateSpeaking:
            display->SetStatus("说话中...");
            ResetDecoder();
            codec->EnableOutput(true);
#if CONFIG_USE_AUDIO_PROCESSING
            audio_processor_.Stop();
#endif
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::SetDecodeSampleRate(int sample_rate) {
    if (opus_decode_sample_rate_ == sample_rate) {
        return;
    }

    opus_decode_sample_rate_ = sample_rate;

    auto opus_codec = Board::GetInstance().GetOpusCodec();
    opus_codec->EncodeConfig(opus_decode_sample_rate_, 1);
}

void Application::UpdateIotStates() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    auto states = thing_manager.GetStatesJson();
    if (states != last_iot_states_) {
        last_iot_states_ = states;
        protocol_->SendIotStates(states);
    }
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    esp_restart();
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (device_state_ == kDeviceStateIdle) {
        ToggleChatState();
        Schedule([this, wake_word]() {
            if (protocol_) {
                protocol_->SendWakeWordDetected(wake_word); 
            }
        }); 
    } else if (device_state_ == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
    } else if (device_state_ == kDeviceStateListening) {   
        if (protocol_) {
            protocol_->CloseAudioChannel();
        }
    }
}
