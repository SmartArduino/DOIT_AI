# ESP32-C3(C2)+VB6824 x 小智 AI 

- 首先致谢虾哥的开源项目：https://github.com/78/xiaozhi-esp32
- 其次致谢：https://github.com/xinnan-tech/xiaozhi-esp32-server

## 项目简介

👉 [视频介绍【bilibili】](https://www.bilibili.com/video/BV1icXPYVEMN/)

👉 [人类：给 AI 装摄像头 vs AI：当场发现主人三天没洗头【bilibili】](https://www.bilibili.com/video/BV1bpjgzKEhd/)

👉 [AI-01模组使用手册](docs/AI-01_使用手册.pdf)

👉 [在线下载固件](https://xiaozhi.doit.am/)

👉 [开发板生产资料](docs/开发板生产资料AI-01-DevKit-v1.1.zip)

👉 [四博Blufi蓝牙配网小智，全小程序操作](https://www.bilibili.com/video/BV1PUTKz8EA7?vd_source=f5bb36b692814a666a5a5d7ea3d5ecad)

特色：

1. VB6824 作为AI语音芯片负责语音打断唤醒和离线语音识别，同时负责录音+音频播放;ESP32-C3(C2)芯片负责接入在线大模型+CozyLife APP。
2. VB6824 UART TX输出降噪后的高信噪比的录音，接到ESP32-C3(C2)芯片UART RX，RX收到的数字音频发给在线大模型。
3. VB6824 从DAC处，做回声信号的采集，接入到PA0/PA1（LINEIN）作为AEC的回采信号。
4. VB6824 语音识别后把识别结果通过UART TX发给ESP32-C3(C2)芯片。



已实现功能

- Wi-Fi / ML307 Cat.1 4G
- 离线语音唤醒 [ESP-SR](https://github.com/espressif/esp-sr)
- 支持两种通信协议（[Websocket](docs/websocket.md) 或 MQTT+UDP）
- 采用 OPUS 音频编解码
- 基于流式 ASR + LLM + TTS 架构的语音交互
- 声纹识别，识别当前说话人的身份 [3D Speaker](https://github.com/modelscope/3D-Speaker)
- OLED / LCD 显示屏，支持表情显示
- 电量显示与电源管理
- 支持多语言（中文、英文、日文）
- 支持 ESP32-C3、ESP32-S3、ESP32-P4 芯片平台
- 通过设备端 MCP 实现设备控制（音量、灯光、电机、GPIO 等）
- 通过云端 MCP 扩展大模型能力（智能家居控制、PC桌面操作、知识搜索、邮件收发等）
- 支持使用小程序配网，自动添加设备码，完全兼容 xiaozhi-esp32-server

## 小程序
* ![小程序码](docs/mini_program.png)


## 软件部分
* ESP-IDF需要在5.4以上，推荐版本为5.4，参考[官方指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c2/get-started/index.html)
* 编译
    ```
    // 热点配网
    idf.py @main/boards/doit-ai-01-kit/build.cfg build   
    // 小程序配网
    idf.py @main/boards/doit-ai-01-kit/build_blufi.cfg build   
    or
    idf.py set-target esp32c2
    idf.py menuconfig
    // Xiaozhi Assistant ---> Board Type ---> Doit-AI-01-Kit    
    idf.py build
    ```

### 下载说明
可使用[配套的下载器](https://item.taobao.com/item.htm?id=903237380382&skuId=5947883431059&spm=a1z10.5-c-s.w4002-21590874298.11.126342baEAq94S)或普通的TTL(USB转串口)
#### 接线指导

##### 下载器
| AI-01开发板     | 下载器 |
| --------- | -------- |
| TX        | RX       |
| RX        | TX       |
| IO9       | BOOT     |
| EN        | EN       |
| 5V        | 5V       |
| GND       | GND      |

##### 普通TTL
| AI-01开发板     | TTL(USB转串口) |
| --------- | -------- |
| TX        | RX       |
| RX        | TX       |
| 3V3       | 3V3      |
| GND       | GND      |

## 下载步骤
- 准备
    - 下载器
        1. 下载器接入电脑即可
    - 普通TTL
        1. AI-01开发板和TTL都断开电脑的连接
        2. 先按住AI-01开发板按键，再把TTL接入电脑，AI-01开发板USB口不要接
- 下载
    - 网页下载    
        1. 浏览器访问https://xiaozhi.doit.am, 并选择四博智联小智AI-01智能体    
        2. 点击烧录，进入烧录界面     
        3. 点击连接，选择弹窗的设备，点击连接    
        4. 点击烧录，等待烧录完成    
    - IDF下载    
        1. idf.py flash   

## 硬件部分

![模组实物](docs/模组实物图.png)

![模组引脚分布图](docs/模组引脚分布图.png)

![模组原理图](docs/模组原理图.png)

## 离线唤醒词
### 唤醒词
| 中文唤醒词           | 英文唤醒词         |
| ------------------ | ---------------- |
| 你好小智            |                  |
| 小艾小艾(需升级支持) | Hey Alice(需升级支持) |

👉 [新手烧录固件教程](https://ccnphfhqs21z.feishu.cn/wiki/Zpz4wXBtdimBrLk25WdcXzxcnNS)

### 机器人

| 中文指令           | 英文指令         |
| ------------------ | ---------------- |
| 再见/不聊了        | Peace out        |
| 站起来/站立        | Stand up         |
| 坐下               | Sit down         |
| 趴下               | Get down         |
| 转个圈             | Turn around      |
| 打个滚             | Roll over        |
| 去尿尿/尿尿去      | Go pee-pee       |
| 去睡觉/睡觉去      | Go to sleep      |
| 装死               | Play dead        |
| 秀一个/跳个舞/跳舞 | Show time        |
| 来个绝活           | Do stunts        |
| 倒立旋转           | Handstand spin   |
| 前进               | Move forward     |
| 后退               | Move backward    |
| 左转/向左转        | Turn left        |
| 右转/向右转        | Turn Right       |
| 过来               | Come here        |
| 走开/滚开/滚蛋     | Go away          |
| 匍匐前进           | Crawling forward |
| 滑步               | Sliding step     |
| 我讨厌你           | I hate you       |

### 灯光

| 中文指令 | 英文指令             |
| -------- | -------------------- |
| 打开灯光 | Turn On The Light    |
| 关闭灯光 | Switch Off The Light |
| 调亮灯光 | Brighten The Light   |
| 调暗灯光 | Dim The Light        |
| 七彩模式 | Colorful Mode        |
| 音乐模式 | Music Mode           |
| 白色灯光 | White Light          |
| 黄色灯光 | Yellow Light         |
| 自然灯光 | Natural Light        |
| 红色灯光 | Red Light            |
| 绿色灯光 | Green Light          |
| 蓝色灯光 | Blue Light           |
| 橙色灯光 | Orange Light         |
| 青色灯光 | Cyan Light           |
| 紫色灯光 | Purple Light         |
|||
### 音乐

| 中文指令 | 英文指令      |
| -------- | ------------- |
| 播放音乐 | Play music    |
| 暂停播放 | Pause playing |
| 停止播放 | Stop playing  |
| 上一首   | Previous song |
| 下一首   | Next song     |

### 配网

| 中文指令 | 英文指令      |
| -------- | ------------- |
| 开始配网 | Start pairing |
| 停止配网 | Stop pairing  |

## 更多问题

技术支持微信：andy433928

模组购买链接：https://item.taobao.com/item.htm?id=901004417223
开发板购买链接：https://item.taobao.com/item.htm?ft=t&id=903237380382
