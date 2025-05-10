#ifndef _EMOTION_REMOTE_H_
#define _EMOTION_REMOTE_H_

#include "gapRemote.h"
#include <functional>
#include <string>

class EmotionRemote {
private:
    struct RemotFun
    {
        std::string funName;
        uint8_t code;
        std::function<void(uint8_t)> func; // 使用 std::function 存储函数
    };

    const uint8_t gapDateLen = 12;
    uint8_t gapData[12] = {
        0x0B,       // 数据长度
        0xFF,       // 厂商自定义类型
        0x77, 0x66, // 厂商 ID
        0x01,       // 协议
        0xFF,       // 标志
        0x95, 0x03, // 设备 ID
        0xFF,       // 组 ID
        0x00,       // 序列号
        0x01,       // 命令
        0x01        // 参数
    };


    GapRemote& remote = GapRemote::GetInstance();
    void sendGapData(uint8_t code);
    public:
    static EmotionRemote& GetInstance() {
        static EmotionRemote instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    EmotionRemote(const EmotionRemote&) = delete;
    EmotionRemote& operator=(const EmotionRemote&) = delete;
    explicit EmotionRemote();
    void SetEmotion(const char* emotion);
};

#endif