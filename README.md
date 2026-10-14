# AI_FACE — 基于 ESP32-S3 的嵌入式人脸识别系统

基于 **ESP32-S3**（16MB Flash，需 PSRAM）+ **ESP-IDF 5.5** 的端侧 AI 实时人脸检测与识别系统，支持人脸注册、识别、数据清空与**陌生人自动报警**，并在 LCD 上实时预览检测结果。

## 功能特性

| 功能 | 触发方式 | 说明 |
|------|---------|------|
| 人脸检测 | 自动（每 5 秒一次） | 两级级联：MSR01 粗检测 → MNP01 精检测（含关键点）；间隔内复用上次结果画框，不重复推理 |
| 人脸注册 | 按键**长按** (>1 秒) | 提取人脸特征（112 维 S16 量化向量）写入 Flash 持久化 |
| 人脸识别（手动） | 按键**单击** | 当场对当前帧检测+识别（不受周期检测限制），LCD 与串口显示结果 |
| 数据清空 | 按键**双击** | 清空 Flash 中所有已注册人脸数据 |
| 实时预览 | 自动 | 摄像头画面（居中 320×240 窗口，带装饰边框）+ 检测框/五点叠加，底部显示状态信息 |
| 陌生人报警 | 自动 | 识别失败时 LCD 红色高亮显示"错误: 陌生人!"，串口同步输出告警日志，并播报语音 audio_04 |
| Welcome 反馈 | 自动 | 识别相似度大于阈值（45%）时，LCD 绿色高亮显示"Welcome: 姓名(相似度%)"，并播报语音 audio_05 |
| 自动识别节流 | 自动 | 每个检测周期（5 秒）自动比对一次，直到识别成功，保证反馈可被看清 |
| 转圈等待动画 | 自动 | 识别/注册推理期间，画面中央显示"彗星弧"等待动画，让推理停顿显得合理 |
| 语音播报 | 自动/按键 | 空库播报 audio_01（提示注册）、注册成功 audio_02、注册失败 audio_03、识别失败 audio_04、识别成功 audio_05 |
| 画面窗口 | 自动 | 摄像头画面 1:1 显示于居中 320×240 窗口（四周留黑 + 青蓝装饰边框） |

## 硬件组成

- **主控**：ESP32-S3（双核，摄像头采集与 LCD 显示运行于 Core1，AI 推理运行于 Core0，语音播报亦运行于 Core1）
- **摄像头**：GC0308，DVP 8 位并口，QVGA 320×240 RGB565 输出，SCCB 复用 I2C0（SDA=GPIO1 / SCL=GPIO2）
- **显示屏**：ST7796 SPI 接口 LCD，480×320 横屏（触摸：FT6X36）
- **IO 扩展**：PCA9557，控制 LCD_CS 与摄像头电源 DVP_PWDN
- **板载外设**：ES8311 音频 Codec + I2S（语音播报，扬声器功放 PA_EN 由 PCA9557 IO1 控制，`voice_player_init` 初始化时拉高使能）、LED、PWM、按键、UART

## 软件架构

### 初始化流程

```
app_main() 初始化链（顺序严格，不可颠倒）:
  i2c_init → pca9557_init → lcd_init → camera_init → app_human_face_recognition()
```

- 摄像头电源由 PCA9557 扩展 IO 第 2 位（DVP_PWDN）控制，`camera_init()` 内部拉低上电。
- 屏幕方向由 `lcd.h` 的 `LCD_ORIENTATION` 宏决定（横屏 480×320 / 竖屏 320×480）。

### 任务流水线

```
task_process_camera (Core1, 优先级5)   摄像头帧采集
        │ xQueueAIFrame（队列长度2）
        ▼
task_process_ai (Core0, 优先级5, 栈8KB)  AI 人脸检测/识别 + 在帧上绘制检测框/五点
                                          人脸检测每 5 秒一次；间隔内复用上次结果画框
        │ xQueueLCDFrame
        ▼
task_process_lcd (Core1, 优先级5)     LCD 显示：居中 320×240 视频窗口 + 装饰边框
                                          + 底部信息栏 + 识别时叠加"转圈等待"动画
        ▲
button_task (Core0, 优先级3)          按键事件（互斥锁保护）→ 注册/识别/清空
voice_task (Core1, 优先级5)           语音播报任务（队列 → I2S → ES8311）
```

### 自动识别与语音反馈机制

两阶段状态机（`RECOGNIZE_COOLDOWN_MS` 等相关宏见 `who_human_face_recognition.cpp` 顶部）：

- **阶段一：人脸库为空**（未注册任何人脸）
  - 不做人脸识别
  - 每隔 **5 秒**（`VOICE_PROMPT_INTERVAL_MS`）循环播报 `audio_01`「请注册人脸」提示用户长按注册
  - 注册成功 → 播报 `audio_02` → 进入阶段二；注册失败 → 播报 `audio_03` 继续提示注册
- **阶段二：人脸库非空**
  - 每 **5 秒**（`AUTO_RECOGNIZE_INTERVAL_MS`）自动比对一次，与检测周期对齐，检测到人脸即启动判断
  - **匹配成功**（相似度 > 阈值 45%）：LCD 绿色 `Welcome: 姓名(相似度%)` + 播报 `audio_05`
  - **匹配失败**（id<0，即未匹配任何已注册者）：判定为**陌生人**，连续 `STRANGER_CONFIRM_FRAMES`(当前1) 次确认后 LCD 红色 `错误: 陌生人!(相似度%)` + 播报 `audio_04`，串口输出 `ESP_LOGW`
  - 判定陌生人不再用相似度门槛过滤（避免真实陌生人因相似度偏低被误判为误检而漏报）；同一陌生人在场期间仅报警一次（`stranger_active` 抑制重播）
- 反馈信息显示 2 秒（`INFO_CLEAR_MS`）后自动清除；告警日志仅在状态切换时输出一次，避免刷屏
- 按键单击识别为**手动识别**：当场对当前帧做一次检测+识别（不受 5 秒周期限制），使用同样反馈（成功 Welcome + audio_05 / 失败错误 + audio_04）；空库时提示"请先注册人脸"并播报 audio_01
- 原有按键功能（长按注册 / 单击识别 / 双击清空）不受影响

### 语音播报

- 播放链路：内嵌 PCM 数据 → `i2s_write_pcm()` → I2S（16kHz/16bit/双声道）→ ES8311 → 喇叭
- 播报接口 `voice_play(voice_id_t)` 为**异步非阻塞**：仅投递请求到队列，由 `voice_task` 实际播放
- 队列满（上一段未播完）时丢弃新请求，且播放前清空积压请求，保证语音与当前状态一致

| 音频文件 | 触发时机 | 说明 |
|---------|---------|------|
| `main/audio/audio_01.pcm` | 人脸库为空 | 提示用户注册人脸 |
| `main/audio/audio_02.pcm` | 人脸注册成功 | 提示进入识别阶段 |
| `main/audio/audio_03.pcm` | 人脸注册失败 | 提示继续注册 |
| `main/audio/audio_04.pcm` | 人脸识别失败 | 提示识别失败 |
| `main/audio/audio_05.pcm` | 人脸识别成功 | 提示识别成功 |

**音频格式要求**（必须与 I2S/ES8311 配置一致）：**16kHz 采样率 / 16bit 位宽 / 双声道立体声 / 原始裸 PCM（无文件头）**。
用 ffmpeg 转换示例：

```bash
ffmpeg -i 语音.wav -ar 16000 -ac 2 -f s16le audio_01.pcm
```

### 画面布局

- 屏幕 480×320；摄像头 QVGA（320×240）画面由 `task_process_lcd` **1:1 显示于居中窗口**（`VIDEO_DISPLAY_W/H` = 320/240，`VIDEO_DISPLAY_X/Y` = 80/40），四周留黑
- **为什么要窗口化**：全屏 480×320 每帧 307KB，80MHz SPI 需约 31ms 写完，超过 ST7796 面板约 17ms 的扫描周期会产生横向撕裂；改为 320×240（153KB≈15ms）后基本消除撕裂，且与摄像头原始分辨率一致、1:1 无需缩放
- 视频窗口四周绘制**装饰边框**（加宽青蓝主带 + 内沿亮线，`draw_video_frame()` 一次性绘制，位于视频之外不受刷新影响）
- **识别/注册推理期间**，画面中央叠加"彗星弧"转圈等待动画（`draw_recognition_spinner()`），把推理停顿表现得像在等待处理
- 视频缓冲从 PSRAM 分配（约 150KB = 320×240×2）
- 底部信息栏（全宽 480×20）位于视频窗口下方，显示注册/识别/清空操作提示与彩色反馈（绿/红/黑底色保证可读）

### 识别阈值配置

- 匹配阈值由宏 `FACE_RECOGNITION_THRESHOLD` 配置（`who_human_face_recognition.cpp`），当前默认 **0.45**（esp-dl 库默认值为 0.55，对 GC0308 摄像头偏严格；已下调以稳定匹配已注册人脸）
- 相似度高于阈值才判定为已注册的同一人，如需进一步放宽可下调至 0.40，但误识别率会升高
- **调试方法**：识别失败时 LCD 会显示最高相似度与阈值（如 `未识别(48% 阈值45%)`），据此微调宏值
- 提高识别率技巧：正面均匀光照（避免逆光）、正脸平视、检测框边长 ≥80 像素、注册与识别时保持相近的光线和距离

### AI 模型（基于 esp-dl 深度学习框架）

| 模型 | 用途 |
|------|------|
| `human_face_detect_msr01` | 人脸粗检测 |
| `human_face_detect_mnp01` | 人脸精检测（含关键点） |
| `face_recognition_112_v1_s16` | 人脸特征提取 / 比对 |

### 数据持久化

人脸特征数据保存在 Flash 分区表 `storage` 分区（SPIFFS，4MB @ 0xC00000），重启后自动加载已注册人脸数据。

## 分区表

| 名称 | 类型 | 子类型 | 偏移 | 大小 |
|------|------|--------|------|------|
| nvs | data | nvs | 0x9000 | 0x6000 |
| phy_init | data | phy | 0xF000 | 0x1000 |
| factory | app | factory | 0x10000 | 0x600000 |
| vfs | data | fat | 0x700000 | 0x400000 |
| storage | data | spiffs | 0xC00000 | 0x400000 |

## 目录结构

```
AI_FACE/
├── main/                                应用代码
│   ├── main.c                           程序入口：硬件初始化
│   ├── who_human_face_recognition.cpp   核心AI应用（注册/识别/清空/陌生人报警逻辑）
│   ├── who_ai_utils.cpp                 AI 绘图工具
│   ├── camera.c / camera.h              DVP 摄像头（GC0308）驱动
│   ├── lcd.c / lcd.h                    ST7796 LCD 驱动（含中文字库绘制）
│   ├── i2c.c / pca9557.c                I2C 总线与 IO 扩展驱动
│   ├── es8311_codec.c / i2s.c           音频 Codec 与 I2S 驱动
│   ├── voice_player.c / voice_player.h  语音播报模块（5段提示音异步播放）
│   ├── key.c / led.c / pwm.c / tp.c / uart.c   按键/LED/PWM/触摸/串口驱动
│   ├── audio/                           语音提示音（audio_01~05.pcm，16kHz/16bit/立体声）
│   └── font/                            中文字库（16/32 点阵）
├── components/
│   └── esp-dl/                          本地 AI 推理框架（Espressif DL）
├── managed_components/                  托管依赖（esp32-camera、ST7796、FT6X36、ES8311 等）
├── partitions.csv                       Flash 分区表
├── sdkconfig / sdkconfig.defaults       SDK 配置
└── CMakeLists.txt                       工程构建入口
```

## 依赖组件（idf_component.yml）
- `espressif/esp32-camera: ^2.1.0` — 摄像头驱动（GC0308 支持）
- `espressif/esp_lcd_st7796: ^1.3.0` — ST7796 LCD 驱动
- `cfscn/esp_lcd_touch_ft6x36: ^1.0.8` — FT6X36 触摸驱动
- `espressif/es8311: ^1.0.0` — ES8311 音频驱动

## 编译与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

### 编译配置要求

- ESP-IDF 5.5.0
- 在 menuconfig 中使能 PSRAM（Octal/Wrap 模式按板卡实际配置）
- Flash 大小配置为 16MB
- 使用自定义分区表 `partitions.csv`
- 语音音频文件必须存在于 `main/audio/`（`audio_01.pcm` ~ `audio_05.pcm`），否则编译报错
  - 仓库内已附带 5 个占位提示音（不同频率的短促蜂鸣），可直接编译验证音频通路，之后用真实语音覆盖同名文件即可

## 注意事项

- AI 推理需要较大栈空间，`task_process_ai` 栈大小设为 8KB
- LCD 显示与 Flash 写入共用总线，需避免同时操作
- 按键事件使用互斥锁保护，保证线程安全
- I2C 必须使用新版驱动（`i2c.h` 中 `I2C_VERSION_2`，配套 IDF 5.5）
