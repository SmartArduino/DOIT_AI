#include "emotion_remote.h"

#include "esp_log.h"

#include "font_awesome_symbols.h"

#define TAG "emzy"

void EmotionRemote::SetEmotion(const char* emotion){
    struct Emotion {
        uint8_t em_id;
        const char* text;
    };
    static const std::vector<Emotion> emotions = {
        {0x01, "neutral"},
        {0x02, "happy"},
        {0x03, "laughing"},
        {0x04, "funny"},
        {0x05, "sad"},
        {0x06, "angry"},
        {0x07, "crying"},
        {0x08, "loving"},
        {0x09, "embarrassed"},
        {0x0a, "surprised"},
        {0x0b, "shocked"},
        {0x0c, "thinking"},
        {0x0d, "winking"},
        {0x0e, "cool"},
        {0x0f, "relaxed"},
        {0x10, "delicious"},
        {0x11, "kissy"},
        {0x12, "confident"},
        {0x13, "sleepy"},
        {0x14, "silly"},
        {0x15, "confused"}
    };
    std::string_view emotion_view(emotion);
    auto it = std::find_if(emotions.begin(), emotions.end(),
        [&emotion_view](const Emotion& e) { return e.text == emotion_view; });
    if (it != emotions.end()) {
        sendGapData(it->em_id);
    } else {
        sendGapData(0x01);
    }
}

void EmotionRemote::sendGapData(uint8_t code){
    gapData[9]++;
    gapData[11] = code;
    remote.start_adv(gapData, gapDateLen, 20000);
}

EmotionRemote::EmotionRemote(){
    
}