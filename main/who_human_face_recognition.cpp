/**
 * @file who_human_face_recognition.cpp
 * @brief 人脸识别应用主程序 — 支持人脸注册、识别和清空
 *
 * ============================================================
 * 使用说明：
 * ============================================================
 * 1. 功能概述：
 *    - 本程序实现基于ESP32-S3的实时人脸检测与识别系统
 *    - 通过摄像头采集视频帧，使用AI模型进行人脸检测和识别
 *    - 支持三种按键操作：长按注册、单击识别、双击清空
 *    - 陌生人监测：人脸库为空时周期播报"请注册人脸"语音(audio_01)；
 *      注册成功后每5秒自动识别一次，相似度大于阈值(50%)绿色显示
 *      "Welcome: 姓名"并播报识别成功(audio_05)，低于阈值红色显示
 *      "错误: 陌生人!"并播报识别失败(audio_04)
 *    - 语音播报：注册成功播报audio_02，注册失败播报audio_03（提示继续注册）
 *    - 画面显示：摄像头画面最近邻放大铺满整屏480x320
 *
 * 2. 硬件要求：
 *    - ESP32-S3开发板（需要PSRAM）
 *    - 摄像头模块（OV2640/OV3660等，支持RGB565输出）
 *    - SPI接口LCD显示屏
 *    - 按键（GPIO11）
 *
 * 3. 操作说明（通过按键操作）：
 *    - 长按(>1秒)：保存当前检测到的人脸数据到Flash分区表
 *    - 单击：识别当前检测到的人脸并在控制台显示结果
 *    - 双击：清空Flash中存储的所有人脸数据（无需检测到人脸）
 *
 * 4. 软件架构：
 *    4个FreeRTOS任务流水线工作：
 *    a) task_process_camera：摄像头帧采集（核心1，优先级5）
 *    b) task_process_ai：AI人脸检测与识别（核心0，优先级5）
 *    c) task_process_lcd：LCD屏幕显示（核心0，优先级5）
 *    d) button_task：按键检测（核心0，优先级3）
 *
 * 5. 数据流：
 *    摄像头 → [xQueueAIFrame] → AI检测 → [xQueueLCDFrame] → LCD显示
 *                                     ↕
 *                              按键事件触发人脸注册/识别/清空
 *
 * 6. 存储说明：
 *    - 人脸数据存储在Flash的DATA分区，子类型ANY，标签"storage"
 *    - 支持写入Flash持久化和从Flash加载已注册的人脸数据
 *    - 分区表需要在menuconfig或partitions.csv中预留足够空间
 *
 * 7. 编译配置：
 *    - ESP-IDF 5.5.0
 *    - 需要在menuconfig中使能PSRAM
 *    - 需要配置摄像头和LCD的引脚
 *    - 需要配置Flash分区表（建议使用factory_8MB或自定义）
 *
 * 8. 注意事项：
 *    - AI推理需要较大栈空间，task_process_ai 栈大小设为8KB
 *    - LCD显示与Flash写入共用SPI总线，需避免同时操作
 *    - 按键事件使用互斥锁保护，线程安全
 *    - 双击清空操作不依赖于摄像头帧数据
 *
 * ============================================================
 */

// ======================== 头文件包含 ========================
#include "who_human_face_recognition.hpp"  // 人脸识别主头文件
#include "dl_image.hpp"                    // 深度学习图像处理库
#include "driver/gpio.h"                   // GPIO驱动
#include "esp_camera.h"                    // ESP32摄像头驱动
#include "esp_log.h"                       // ESP-IDF日志系统
#include "face_recognition_112_v1_s16.hpp" // 人脸识别器（S16量化）
#include "face_recognition_tool.hpp"       // 人脸识别工具函数
#include "human_face_detect_mnp01.hpp"     // MNP01人脸精检测器
#include "human_face_detect_msr01.hpp"     // MSR01人脸粗检测器
#include "lcd.h"                           // 本工程 LCD 模块：LCD_WIDTH/LCD_HEIGHT 与显示接口
#include "who_ai_utils.hpp"                // AI通用工具函数
#include <stdarg.h>                        // 可变参数接口：va_list/vsnprintf
#include <string.h>                        // C字符串接口：strlen/strncpy/strcmp
#include <string>                          // C++标准字符串
#include "esp_heap_caps.h"                 // PSRAM堆内存分配接口
#include "voice_player.h"                  // 语音播报模块（提示注册/注册/识别结果播报）

// ======================== LCD 常用颜色定义 ========================
/**
 * 颜色校准说明：
 * 本面板显示链路存在颜色通道循环错位（实测 0xF800 显示为蓝、0x07E0 显示为红，
 * 即 R→B、G→R、B→G 错位），此处宏值按实测/推算校准，使宏名与实际显示颜色一致：
 *   - RED   = 0x07E0（实测显示红色）
 *   - GREEN = 0x001F（按通道循环规律推算显示绿色）
 * 白/黑两色（全0/全1）不受通道错位影响，保持标准值。
 * 若绿色显示仍不正确，请告知实际看到的颜色以便再次校准。
 */
#define WHITE 0xFFFF // 白色
#define BLACK 0x0000 // 黑色
#define RED 0x07E0   // 红色（校准值，陌生人/错误提示背景色）
#define GREEN 0x001F // 绿色（校准值，Welcome欢迎反馈背景色）

// ======================== 日志与全局变量 ========================

/**
 * @var TAG
 * @brief 日志标签，用于ESP-IDF日志系统输出识别信息
 *
 * 所有日志输出使用此标签，便于在串口监视器中过滤
 */
static const char* TAG = "human_face_recognition";

/**
 * @var xQueueLCDFrame
 * @brief LCD显示帧队列句柄
 *
 * 用于在AI处理任务和LCD显示任务之间传递已处理的帧数据
 * 队列元素类型：camera_fb_t *（帧缓冲区指针）
 * 队列长度：2
 */
static QueueHandle_t xQueueLCDFrame = NULL;

/**
 * @var xQueueAIFrame
 * @brief AI处理帧队列句柄
 *
 * 用于在摄像头采集任务和AI处理任务之间传递原始帧数据
 * 队列元素类型：camera_fb_t *（帧缓冲区指针）
 * 队列长度：2
 */
static QueueHandle_t xQueueAIFrame = NULL;

/**
 * @var recognizer
 * @brief 人脸识别器实例指针
 *
 * 使用FaceRecognition112V1S16类，基于S16量化模型
 * 负责人脸特征提取、注册、识别、删除等操作
 */
static FaceRecognition112V1S16* recognizer = NULL;

/**
 * @var g_recognizing
 * @brief 是否正在进行人脸识别/注册的 AI 推理
 *
 * AI 任务在调用 recognizer->recognize()/enroll_id() 前置 true、完成后置 false。
 * LCD 显示任务据此在画面右上角叠加"转圈等待"动画，让推理期间的画面停顿
 * 看起来是在等待处理。推理运行在 Core0，动画任务在 Core1，互不阻塞。
 */
static volatile bool g_recognizing = false;

// ======================== LCD关键信息显示定义 ========================

/**
 * @def LCD_INFO_X
 * @brief 关键信息显示起始X坐标（屏幕底部）
 *
 * 摄像头图像高240px，LCD总高320px，信息显示在底部20px区域
 * 使用全宽480px显示文字
 */
#define LCD_INFO_X (0) // 信息显示起始X坐标

/**
 * @def LCD_INFO_Y
 * @brief 关键信息显示起始Y坐标（摄像头图像下方）
 */
#define LCD_INFO_Y (LCD_HEIGHT - 20) // 信息显示起始Y坐标 (300)

/**
 * @def LCD_INFO_W
 * @brief 关键信息显示区域宽度
 */
#define LCD_INFO_W (LCD_WIDTH) // 信息显示区域宽度 (480)

/**
 * @def LCD_INFO_FONT
 * @brief 关键信息字体大小
 */
#define LCD_INFO_FONT 16 // 字体大小

/**
 * @def LCD_INFO_H
 * @brief 信息显示区域高度
 */
#define LCD_INFO_H (20) // 信息显示区域高度

/**
 * @def LCD_INFO_TEXT_Y_OFFSET
 * @brief 字体在显示区域内垂直居中的Y偏移量
 *
 * 区域高20px，字体大小16px，偏移 (20-16)/2 = 2px
 */
#define LCD_INFO_TEXT_Y_OFFSET ((LCD_INFO_H - LCD_INFO_FONT) / 2) // 字体垂直居中偏移

/**
 * @def INFO_CLEAR_MS
 * @brief 信息自动清除时间（毫秒）
 *
 * 信息显示后，超过此时间自动清除
 */
#define INFO_CLEAR_MS (2000) // 自动清除时间(2秒)

/**
 * @def AUTO_RECOGNIZE_INTERVAL_MS
 * @brief 自动识别周期（毫秒）
 *
 * 识别失败后，每隔该时间自动进行一次人脸比对，直到识别成功。
 * 反馈（Welcome/错误提示+语音播报）显示 INFO_CLEAR_MS(3秒) 后自动消失。
 */
#define AUTO_RECOGNIZE_INTERVAL_MS (5000) // 与检测周期对齐，每5秒做一次人脸识别

/**
 * @def VOICE_PROMPT_INTERVAL_MS
 * @brief 人脸库为空时，注册提示语音的播报间隔（毫秒）
 *
 * 库为空时周期播报"请注册人脸"提示音(audio_01)，避免连续重复播报。
 */
#define VOICE_PROMPT_INTERVAL_MS (5000) // 空库注册提示每5秒播报一次

/**
 * @def DETECT_INTERVAL_MS
 * @brief 人脸检测周期
 *
 * 实际运行时按此间隔做一次 MTCNN + MNP01 双阶段检测，间隔内的帧
 * 复用上一次的检测结果画框，不做新的 AI 推理。
 *
 * 设计理由：
 *   - 避免长时间占摄像头时每帧推理浪费 CPU（每帧 100~300ms 推理 × 30fps = 占满）
 *   - 避免每帧持续触发 voice 队列导致 I2S 欠喂/卡死
 *   - 与 AUTO_RECOGNIZE_INTERVAL_MS(5s) 配合形成"5s 一次检测+识别"闭环
 *   - 无人在场时完全不触发任何语音
 */
#define DETECT_INTERVAL_MS (5000) // 每5秒做一次检测+识别：与识别周期对齐，间隔内复用上次检测结果画框，进一步降低卡顿频率

/**
 * @def VIDEO_DISPLAY_W / VIDEO_DISPLAY_H / VIDEO_DISPLAY_X / VIDEO_DISPLAY_Y
 * @brief 摄像头画面在 LCD 上的显示尺寸与位置（居中窗口，非全屏）
 *
 * 采用 320×240 居中窗口显示（四周留黑）。原因：全屏 480×320 每帧 307KB，
 * 80MHz SPI 需约 31ms 才能写完，超过 ST7796 面板约 17ms 的扫描周期，会产生
 * 横向撕裂（接缝/水波纹）。改为 320×240（153KB≈15ms，小于一个扫描周期）后可
 * 基本消除撕裂；且与摄像头原始 QVGA 尺寸一致，1:1 显示无需缩放。
 *
 *   屏幕布局示意（480×320）：
 *   ┌──────────────────────────────────┐
 *   │            黑边(40px)             │
 *   │     ┌────────────────────┐        │
 *   │     │  视频 320×240 (居中) │        │
 *   │     └────────────────────┘        │
 *   │            黑边(40px)             │
 *   ├──────────────────────────────────┤
 *   │  底部信息栏: Welcome/陌生人/提示    │
 *   │          480×20 (全宽)           │
 *   └──────────────────────────────────┘
 */
#define VIDEO_DISPLAY_W (320)                                 // 视频宽度（=QVGA 宽，1:1）
#define VIDEO_DISPLAY_H (240)                                 // 视频高度（=QVGA 高，1:1）
#define VIDEO_DISPLAY_X ((LCD_WIDTH - VIDEO_DISPLAY_W) / 2)   // 水平居中 (80)
#define VIDEO_DISPLAY_Y ((LCD_HEIGHT - VIDEO_DISPLAY_H) / 2)  // 垂直居中 (40)

/**
 * @def FACE_RECOGNITION_THRESHOLD
 * @brief 人脸识别匹配阈值
 *
 * 相似度高于该阈值才判定为已注册的同一人，默认0.55（esp-dl库默认值）。
 * GC0308摄像头画质有限，光照/角度变化时本人相似度可能只有0.45~0.55，
 * 若识别困难可适当下调（如0.45），误识别率会相应升高。
 * 调试方法：识别失败时LCD会显示最高相似度，据此微调本值。
 */
#define FACE_RECOGNITION_THRESHOLD (0.45f)

/**
 * @def STRANGER_SIMILARITY_FLOOR
 * @brief 识别失败时报警的最低相似度门槛
 *
 * GC0308 + MTCNN 在空场景中偶发误检（光照变化、移动纹理等），识别器
 * 输出 id<0 的同时 similarity 也往往极低（<0.20）。把"看起来完全不像
 * 已注册任何人的脸"区分为"压根不像人脸"，低于本门槛直接跳过——
 * 这是"识别不到人脸也会误报陌生人"问题的根治办法。
 *
 * 调参建议：
 *   - 0.20~0.25：宽松，接受更多低质量检测结果为"陌生人"
 *   - 0.30      ：推荐默认；明显小于已注册者的陌生面孔仍能告警
 *   - 0.40      ：严格，必须与已注册人脸有较高相似度才告警
 */
#define STRANGER_SIMILARITY_FLOOR (0.30f)

/**
 * @def STRANGER_CONFIRM_FRAMES
 * @brief 连续 N 次识别失败（每5秒一次）才认定为陌生人并报警
 *
 * 识别阶段每 AUTO_RECOGNIZE_INTERVAL_MS(5s) 执行一次：
 *   - 匹配成功(id>=0)：Welcome + 播报 audio_05，复位陌生人状态
 *   - 识别失败(id<0)：判定为陌生人候选，计数+1
 *   - 连续达到本帧数：确认陌生人 → LCD 红色"错误: 陌生人!" + 播报 audio_04
 * 注：不再用 STRANGER_SIMILARITY_FLOOR 过滤低相似度，否则真陌生人（相似度本就偏低）
 *     会被误判为检测器误检而漏报。设为 1 即"识别一次失败立即报警"（三秒判断一次）。
 * 同一陌生人在场期间只报警一次（stranger_active 抑制重播）。
 */
#define STRANGER_CONFIRM_FRAMES (1)

/**
 * @def STRANGER_LOG_COOLDOWN_MS
 * @brief 陌生人日志的输出间隔（防止 ESP_LOGW 刷屏）
 */
#define STRANGER_LOG_COOLDOWN_MS (3000)

/**
 * @struct lcd_info_msg_t
 * @brief LCD关键信息显示结构体
 *
 * 用于在task_process_ai和task_process_lcd之间传递需要显示的文本信息
 * 通过互斥锁保护，实现线程安全的读写操作
 */
typedef struct
{
    char text[64];        /**< 要显示的文本内容 */
    TickType_t timestamp; /**< 设置时的时间戳（用于超时清除） */
    bool active;          /**< 是否有效显示标志 */
    uint16_t fc;          /**< 文字前景色（RGB565），默认白色 */
    uint16_t bc;          /**< 文字背景色（RGB565），默认黑色，警告时为红色 */
} lcd_info_msg_t;

/**
 * @var g_lcd_info
 * @brief 全局LCD关键信息变量
 *
 * 在AI任务中设置，在LCD显示任务中读取显示
 * 通过互斥锁保护并发访问
 */
static lcd_info_msg_t g_lcd_info = {"", 0, false, WHITE, BLACK};

/**
 * @var lcd_info_mutex
 * @brief LCD关键信息互斥锁
 *
 * 保护g_lcd_info的并发访问
 * 在task_process_ai（写）和task_process_lcd（读/写）之间提供同步
 */
static SemaphoreHandle_t lcd_info_mutex = NULL;

/**
 * @brief 设置LCD关键信息的内部实现（va_list版本，线程安全）
 *
 * 由set_lcd_info / set_lcd_warn包装调用，统一实现写入逻辑
 *
 * @param fc  文字前景色（RGB565）
 * @param bc  文字背景色（RGB565）
 * @param fmt 格式化字符串（支持printf格式）
 * @param args 已初始化的va_list参数
 * @return 无返回值
 */
static void set_lcd_info_va(uint16_t fc, uint16_t bc, const char* fmt, va_list args)
{
    char temp_buf[64];

    // 格式化文本
    vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);

    // 线程安全地写入全局信息结构体
    if (xSemaphoreTake(lcd_info_mutex, portMAX_DELAY) == pdTRUE)
    {
        // 复制文本内容（防止缓冲区溢出）
        strncpy(g_lcd_info.text, temp_buf, sizeof(g_lcd_info.text) - 1);
        g_lcd_info.text[sizeof(g_lcd_info.text) - 1] = '\0';
        // 记录当前时间戳，用于3秒自动清除
        g_lcd_info.timestamp = xTaskGetTickCount();
        // 激活显示标志
        g_lcd_info.active = true;
        // 设置显示颜色
        g_lcd_info.fc = fc;
        g_lcd_info.bc = bc;
        // 释放互斥锁
        xSemaphoreGive(lcd_info_mutex);
    }
}

/**
 * @brief 设置LCD关键信息（白字黑底，线程安全）
 *
 * 将格式化的文本信息写入全局lcd_info结构体，并激活显示标志。
 * 由task_process_ai在按键事件或识别结果处理时调用。
 * LCD显示任务会读取此信息并在摄像头图像下方显示。
 *
 * @param fmt 格式化字符串（支持printf格式）
 * @param ... 可变参数（与fmt对应）
 * @return 无返回值
 *
 * @note 文本最大长度63字节，超出部分将被截断
 * @note 此函数会更新时间戳，重置3秒清除计时
 */
static void set_lcd_info(const char* fmt, ...)
{
    // 检查互斥锁是否已初始化
    if (!lcd_info_mutex)
        return;

    va_list args;
    va_start(args, fmt);
    set_lcd_info_va(WHITE, BLACK, fmt, args);
    va_end(args);
}

/**
 * @brief 设置LCD陌生人警告信息（白字红底高亮，线程安全）
 *
 * 与set_lcd_info功能相同，但以红色背景、白色文字高亮显示，
 * 用于陌生人监测报警。LCD显示任务按g_lcd_info中的颜色绘制。
 *
 * @param fmt 格式化字符串（支持printf格式）
 * @param ... 可变参数（与fmt对应）
 * @return 无返回值
 *
 * @note 文本最大长度63字节，超出部分将被截断
 * @note 此函数会更新时间戳，重置3秒清除计时
 */
static void set_lcd_warn(const char* fmt, ...)
{
    // 检查互斥锁是否已初始化
    if (!lcd_info_mutex)
        return;

    va_list args;
    va_start(args, fmt);
    set_lcd_info_va(WHITE, RED, fmt, args);
    va_end(args);
}

/**
 * @brief 设置LCD欢迎反馈信息（白字绿底高亮，线程安全）
 *
 * 人脸识别匹配成功（相似度大于阈值）时调用，
 * 以绿色背景、白色文字高亮显示"Welcome"欢迎信息。
 *
 * @param fmt 格式化字符串（支持printf格式）
 * @param ... 可变参数（与fmt对应）
 * @return 无返回值
 *
 * @note 文本最大长度63字节，超出部分将被截断
 * @note 此函数会更新时间戳，重置3秒清除计时
 */
static void set_lcd_welcome(const char* fmt, ...)
{
    // 检查互斥锁是否已初始化
    if (!lcd_info_mutex)
        return;

    va_list args;
    va_start(args, fmt);
    set_lcd_info_va(WHITE, GREEN, fmt, args);
    va_end(args);
}

// ======================== 按键相关定义 ========================

/**
 * @def BUTTON_GPIO
 * @brief 按键GPIO引脚定义
 *
 * 保持一致，按键连接到GPIO11
 * 按下时电平为低（0），释放时电平为高（1）
 */
#define BUTTON_GPIO GPIO_NUM_11

/**
 * @enum button_event_t
 * @brief 按键事件类型枚举
 *
 * 定义支持的按键操作事件类型
 * 由button_task检测后设置，由task_process_ai消费处理
 */
typedef enum
{
    BUTTON_EVENT_NONE = 0,     /**< 无按键事件，默认状态 */
    BUTTON_EVENT_SINGLE_CLICK, /**< 单击事件：识别人脸 */
    BUTTON_EVENT_DOUBLE_CLICK, /**< 双击事件：清空所有人脸数据 */
    BUTTON_EVENT_LONG_PRESS,   /**< 长按事件：注册人脸 */
} button_event_t;

/**
 * @enum button_state_t
 * @brief 按键状态机状态定义
 *
 * 按键检测使用有限状态机实现，包含4个状态
 * 确保准确的单击、双击和长按识别
 */
typedef enum
{
    BUTTON_STATE_IDLE,         /**< 空闲状态：等待按键按下 */
    BUTTON_STATE_PRESSED,      /**< 按下状态：检测释放或长按 */
    BUTTON_STATE_WAIT_RELEASE, /**< 等待释放状态：长按后等待松开 */
    BUTTON_STATE_WAIT_DOUBLE,  /**< 等待双击状态：检测第二次点击 */
} button_state_t;

/**
 * @var g_button_event
 * @brief 全局按键事件标志
 *
 * 使用volatile修饰，确保在中断上下文和任务间访问的一致性
 * 通过互斥锁保护，实现线程安全的读写操作
 */
static volatile button_event_t g_button_event = BUTTON_EVENT_NONE;

/**
 * @var button_mutex
 * @brief 按键事件互斥锁
 *
 * 保护g_button_event的并发访问
 * 在button_task（写）和task_process_ai（读/写）之间提供同步
 */
static SemaphoreHandle_t button_mutex = NULL;

// ======================== 函数前向声明 ========================
static button_event_t get_button_event(void);
static button_event_t peek_button_event(void);

// =================================================================
// 按键事件管理函数
// =================================================================

/**
 * @brief 获取当前按键事件（消费模式）
 *
 * 线程安全地获取当前检测到的按键事件，并立即清除事件标志。
 * 调用此函数后，按键事件标志被重置为BUTTON_EVENT_NONE，
 * 防止同一事件被重复处理。
 *
 * @return button_event_t 返回检测到的按键事件类型
 *         - BUTTON_EVENT_NONE: 无按键事件
 *         - BUTTON_EVENT_SINGLE_CLICK: 单击事件
 *         - BUTTON_EVENT_DOUBLE_CLICK: 双击事件
 *         - BUTTON_EVENT_LONG_PRESS: 长按事件
 *
 * @note 如果互斥锁获取失败（极端情况），返回BUTTON_EVENT_NONE
 * @warning 此函数会清除事件标志，每次事件只会被消费一次
 */
static button_event_t get_button_event(void)
{
    // 初始化返回值为无事件
    button_event_t event = BUTTON_EVENT_NONE;

    /**
     * 获取互斥锁，确保对共享变量g_button_event的原子访问
     * portMAX_DELAY表示无限等待，直到获取到互斥锁
     */
    if (xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE)
    {
        // 读取当前按键事件值
        event = g_button_event;
        // 清除事件标志，防止重复处理
        g_button_event = BUTTON_EVENT_NONE;
        // 释放互斥锁
        xSemaphoreGive(button_mutex);
    }

    // 返回获取到的按键事件类型
    return event;
}

/**
 * @brief 查看当前按键事件（非消费模式）
 *
 * 线程安全地查看当前检测到的按键事件，但不清除事件标志。
 * 用于在不消费事件的情况下检查是否有按键事件发生，
 * 常用于非阻塞的事件轮询场景。
 *
 * @return button_event_t 返回当前检测到的按键事件类型
 *         - BUTTON_EVENT_NONE: 无按键事件
 *         - BUTTON_EVENT_SINGLE_CLICK: 单击事件
 *         - BUTTON_EVENT_DOUBLE_CLICK: 双击事件
 *         - BUTTON_EVENT_LONG_PRESS: 长按事件
 *
 * @note 此函数不会清除事件标志，事件仍然可以被后续的get_button_event()消费
 * @note 如果互斥锁获取失败，返回BUTTON_EVENT_NONE
 */
static button_event_t peek_button_event(void)
{
    // 初始化返回值为无事件
    button_event_t event = BUTTON_EVENT_NONE;

    // 获取互斥锁，读取当前按键事件值但不清除
    if (xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE)
    {
        // 仅读取事件值，不清除标志
        event = g_button_event;
        // 释放互斥锁
        xSemaphoreGive(button_mutex);
    }

    // 返回当前按键事件类型
    return event;
}

/**
 * @brief 设置按键事件
 *
 * 线程安全地设置按键事件标志。
 * 由button_task按键检测任务调用，将检测到的事件类型
 * 写入全局变量，供task_process_ai处理。
 *
 * @param event 要设置的按键事件类型
 *         - BUTTON_EVENT_SINGLE_CLICK: 单击事件
 *         - BUTTON_EVENT_DOUBLE_CLICK: 双击事件
 *         - BUTTON_EVENT_LONG_PRESS: 长按事件
 * @return 无返回值
 *
 * @note 此函数会覆盖之前未处理的事件
 * @note 如果互斥锁获取失败，事件设置失败（不会阻塞）
 */
static void set_button_event(button_event_t event)
{
    // 获取互斥锁，保护对g_button_event的写入操作
    if (xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE)
    {
        // 设置按键事件值
        g_button_event = event;
        // 释放互斥锁
        xSemaphoreGive(button_mutex);
    }
    /**
     * 注意：如果互斥锁获取失败，事件将不会被设置。
     * 在正常系统负载下，portMAX_DELAY等待不应超时。
     */
}

// =================================================================
// 按键检测任务（button_task）
// =================================================================

/**
 * @brief 按键检测任务
 *
 * 该任务负责检测按键的状态，通过四状态有限状态机
 * 实现单击、双击和长按的准确识别。
 * 使用软件消抖机制（20ms延时）确保按键检测的可靠性。
 *
 * 状态机说明：
 *   BUTTON_STATE_IDLE         → 按下 → BUTTON_STATE_PRESSED
 *   BUTTON_STATE_PRESSED      → 释放 → BUTTON_STATE_WAIT_DOUBLE
 *   BUTTON_STATE_PRESSED      → 长按超时 → 触发长按 → WAIT_RELEASE
 *   BUTTON_STATE_WAIT_DOUBLE  → 再次按下 → BUTTON_STATE_PRESSED
 *   BUTTON_STATE_WAIT_DOUBLE  → 超时 → 判定单击/双击 → IDLE
 *   BUTTON_STATE_WAIT_RELEASE → 释放 → BUTTON_STATE_IDLE
 *
 * @param arg 任务参数指针（本实现中未使用，传入NULL）
 * @return 无返回值（任务循环运行，永不返回）
 *
 * @note 任务栈大小：4KB（修复了原2KB栈溢出问题）
 * @note 优先级：3（较低优先级，避免影响AI和显示任务）
 * @note 运行核心：0（应用核心）
 * @note 轮询周期：10ms（vTaskDelay）
 * @note 消抖时间：20ms（软件延时去抖动）
 * @note 长按阈值：1000ms
 * @note 双击间隔：300ms
 */
static void button_task(void* arg)
{
    // ---- 初始化按键GPIO配置 ----

    /**
     * 创建GPIO配置结构体并初始化为零
     * gpio_config_t包含中断类型、模式、引脚掩码、上拉/下拉配置
     */
    gpio_config_t io_conf = {};

    // 禁用中断，采用轮询方式读取按键状态
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // 将GPIO设置为输入模式
    io_conf.mode = GPIO_MODE_INPUT;

    /**
     * 设置引脚位掩码
     * 将1左移BUTTON_GPIO位，选中对应的GPIO引脚
     * 例如GPIO_NUM_0，则bit0=1
     */
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);

    // 使能内部上拉电阻，确保按键未按下时引脚为高电平
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    // 禁用下拉电阻
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    // 将配置应用到GPIO控制器
    gpio_config(&io_conf);

    // ---- 状态机变量初始化 ----

    // 按键状态机初始状态：空闲等待
    button_state_t state = BUTTON_STATE_IDLE;

    // 记录按键按下的系统Tick计数
    TickType_t press_time = 0;

    // 记录按键释放的系统Tick计数
    TickType_t release_time = 0;

    // 点击次数计数器（用于区分单击和双击）
    int click_count = 0;

    // ---- 按键检测参数定义 ----

    /**
     * 软件消抖时间：20ms
     * 在检测到电平变化后延时20ms再次确认，避免机械抖动误触发
     */
    const TickType_t DEBOUNCE_TIME = pdMS_TO_TICKS(20);

    /**
     * 长按判定阈值：1000ms（1秒）
     * 按键持续按下超过此时间则触发长按事件
     */
    const TickType_t LONG_PRESS_TIME = pdMS_TO_TICKS(1000);

    /**
     * 双击判定间隔：300ms
     * 两次按键释放之间的间隔小于此值判定为双击
     * 超过此值则判定为单击
     */
    const TickType_t DOUBLE_CLICK_TIME = pdMS_TO_TICKS(300);

    // 打印按键检测任务启动信息和操作说明
    ESP_LOGI(TAG, "按键检测任务已启动，等待按键操作...");
    ESP_LOGI(TAG, "操作说明：");
    ESP_LOGI(TAG, "  - 长按(>1秒)：保存人脸数据到分区表");
    ESP_LOGI(TAG, "  - 单击：识别人脸并显示结果");
    ESP_LOGI(TAG, "  - 双击：清空所有人脸数据");

    // ---- 任务主循环 ----
    while (true)
    {
        /**
         * 读取当前按键电平状态
         * 按键默认上拉，按下时GPIO电平为低（0），释放时为高（1）
         */
        int level = gpio_get_level(BUTTON_GPIO);

        // 按键状态机处理
        switch (state)
        {

            // ==================== 状态1：空闲状态 ====================
            case BUTTON_STATE_IDLE:
                // 检测到按键按下（电平为低）
                if (level == 0)
                {
                    // 延时20ms进行软件消抖
                    vTaskDelay(DEBOUNCE_TIME);

                    // 消抖后再次确认电平状态
                    if (gpio_get_level(BUTTON_GPIO) == 0)
                    {
                        // 确认按键已按下，切换到按下状态
                        state = BUTTON_STATE_PRESSED;
                        // 记录当前Tick作为按下时间
                        press_time = xTaskGetTickCount();
                    }
                }
                break;

            // ==================== 状态2：按下状态 ====================
            case BUTTON_STATE_PRESSED:
                // 检测到按键释放（电平为高）
                if (level == 1)
                {
                    // 延时20ms消抖
                    vTaskDelay(DEBOUNCE_TIME);

                    // 消抖后确认电平为高（已释放）
                    if (gpio_get_level(BUTTON_GPIO) == 1)
                    {
                        // 记录释放时间，用于后续双击间隔计算
                        release_time = xTaskGetTickCount();
                        // 点击次数加1
                        click_count++;
                        // 进入等待双击状态，检测是否有第二次点击
                        state = BUTTON_STATE_WAIT_DOUBLE;
                    }
                }
                else if ((xTaskGetTickCount() - press_time) > LONG_PRESS_TIME)
                {
                    /**
                     * 按键持续按下超过长按阈值（1秒）
                     * 判定为长按事件，立即触发
                     */
                    set_button_event(BUTTON_EVENT_LONG_PRESS);
                    ESP_LOGI(TAG, "检测到长按事件");
                    // 进入等待释放状态，等待按键松开
                    state = BUTTON_STATE_WAIT_RELEASE;
                }
                break;

            // ==================== 状态3：等待释放状态 ====================
            case BUTTON_STATE_WAIT_RELEASE:
                // 检测到按键释放（电平为高）
                if (level == 1)
                {
                    // 延时20ms消抖
                    vTaskDelay(DEBOUNCE_TIME);

                    // 确认按键已释放
                    if (gpio_get_level(BUTTON_GPIO) == 1)
                    {
                        // 回到空闲状态，等待下一次按键
                        state = BUTTON_STATE_IDLE;
                    }
                }
                break;

            // ==================== 状态4：等待双击状态 ====================
            case BUTTON_STATE_WAIT_DOUBLE:
                // 检测到第二次按键按下（即将产生双击）
                if (level == 0)
                {
                    // 延时20ms消抖
                    vTaskDelay(DEBOUNCE_TIME);

                    // 确认第二次按下有效
                    if (gpio_get_level(BUTTON_GPIO) == 0)
                    {
                        // 点击次数加1（此时click_count为2）
                        click_count++;
                        // 回到按下状态，等待第二次释放
                        state = BUTTON_STATE_PRESSED;
                        // 更新按下时间
                        press_time = xTaskGetTickCount();
                    }
                }
                else if ((xTaskGetTickCount() - release_time) > DOUBLE_CLICK_TIME)
                {
                    /**
                     * 超过双击判定间隔（300ms），未检测到第二次点击
                     * 根据click_count确定事件类型：
                     *   - click_count == 1：单击事件
                     *   - click_count >= 2：双击事件
                     */
                    if (click_count == 1)
                    {
                        // 触发单击事件
                        set_button_event(BUTTON_EVENT_SINGLE_CLICK);
                        ESP_LOGI(TAG, "检测到单击事件");
                    }
                    else if (click_count >= 2)
                    {
                        // 触发双击事件
                        set_button_event(BUTTON_EVENT_DOUBLE_CLICK);
                        ESP_LOGI(TAG, "检测到双击事件");
                    }

                    // 重置点击计数器和状态
                    click_count = 0;
                    state = BUTTON_STATE_IDLE;
                }
                break;

            // 默认情况（不应发生）
            default:
                break;
        }

        /**
         * 任务延时10ms
         * 控制轮询频率，避免过度占用CPU资源
         * 同时为其他任务提供调度机会
         */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =================================================================
// 结果显示函数
// =================================================================

/**
 * @brief 在LCD上显示人脸识别结果（通过串口日志）
 *
 * 根据识别结果（姓名和相似度）在控制台输出相应的日志信息。
 * 如果识别成功，显示姓名和相似度分数；
 * 如果识别失败，提示未匹配到已知人脸。
 * 该版本的实现通过ESP_LOG输出到串口，未使用LCD图形绘制。
 *
 * @param name       识别到的人脸姓名（C字符串）
 * @param similarity 相似度分数（范围0.0~1.0，越接近1.0越匹配）
 * @param x          LCD显示位置的X坐标（当前版本未使用）
 * @param y          LCD显示位置的Y坐标（当前版本未使用）
 * @return 无返回值
 *
 * @note 当前版本仅通过串口输出结果，LCD显示仅用于摄像头预览
 * @note name参数可以为NULL或空字符串，此时显示"识别失败"
 */
static void display_recognition_result(const char* name, float similarity, int x, int y)
{
    // 检查name是否有效且非空
    if (name && strlen(name) > 0)
    {
        // 识别成功：打印姓名和相似度
        ESP_LOGI(TAG, "识别成功！姓名: %s, 相似度: %.2f", name, similarity);
    }
    else
    {
        // 识别失败：未匹配到任何人脸
        ESP_LOGI(TAG, "识别失败！未匹配到已知人脸");
    }
}

// =================================================================
// AI人脸检测与识别任务（task_process_ai）
// =================================================================

/**
 * @brief AI人脸检测与识别主任务
 *
 * 该任务是系统的核心处理单元，负责：
 * 1. 从AI帧队列接收摄像头采集的帧数据
 * 2. 使用两级检测器（MSR01 + MNP01）进行人脸检测
 * 3. 将检测结果绘制到帧数据上
 * 4. 将处理后的帧发送到LCD显示队列
 * 5. 根据按键事件执行人脸注册、识别、清空操作
 *
 * 执行流程：
 *   peek按键事件 → 接收帧 → 人脸检测 → LCD显示 → 处理按键事件
 *                                                     ↓
 *                                           长按: 注册人脸到Flash
 *                                           单击: 识别当前人脸
 *                                           双击: 清空所有人脸
 *
 * @param arg 任务参数指针（本实现中未使用，传入NULL）
 * @return 无返回值（任务循环运行，永不返回）
 *
 * @note 任务栈大小：8KB（需要较大栈空间用于神经网络推理）
 * @note 优先级：5
 * @note 运行核心：0（应用核心）
 * @note 帧队列接收超时：100ms，确保按键事件能被及时响应
 * @note LCD显示优先于Flash写入，避免SPI总线冲突导致黑屏
 */

// =================================================================
// AI人脸检测与识别任务（task_process_ai）
// =================================================================

static void task_process_ai(void* arg)
{
    // ---- 局部变量声明 ----

    /**
     * 帧缓冲区指针
     * 从xQueueAIFrame接收摄像头帧数据
     * 初始为NULL，接收到有效帧后指向帧缓冲区
     */
    camera_fb_t* frame = NULL;

    // ---- AI模型初始化 ----

    /**
     * 创建MSR01人脸检测器实例（第一级：粗检测）
     * 功能：快速扫描全图，生成人脸候选区域
     * 参数说明：
     *   - 0.3F：检测分数阈值（低于此分数的候选被丢弃）
     *   - 0.3F：NMS的IOU阈值（重叠超过此值的候选被合并）
     *   - 10：最大候选框数量
     *   - 0.3F：分数阈值（最终过滤用）
     */
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);

    /**
     * 创建MNP01人脸检测器实例（第二级：精检测）
     * 功能：在MSR01的候选区域上进行精确检测，输出精确边界框和关键点
     * 参数说明：
     *   - 0.35F：检测分数阈值（比MSR01更宽松，提高小脸/侧脸检出率）
     *   - 0.3F：NMS的IOU阈值
     *   - 10：最大检测结果数量
     */
    HumanFaceDetectMNP01 detector2(0.35F, 0.3F, 10);

    /**
     * 创建人脸识别器实例
     * FaceRecognition112V1S16：基于S16量化的轻量级人脸识别模型
     * 支持人脸特征提取、注册、识别和删除功能
     */
    recognizer = new FaceRecognition112V1S16();

    /**
     * 设置人脸识别匹配阈值
     * esp-dl库默认阈值为0.55，对GC0308摄像头画质偏严格，
     * 此处通过宏FACE_RECOGNITION_THRESHOLD配置（默认0.50）
     */
    recognizer->set_thresh(FACE_RECOGNITION_THRESHOLD);
    ESP_LOGI(TAG, "人脸识别阈值: %.2f", recognizer->get_thresh());

    /**
     * 配置Flash分区用于存储人脸特征数据
     * 分区类型：ESP_PARTITION_TYPE_DATA（数据分区）
     * 子类型：ESP_PARTITION_SUBTYPE_ANY（任意子类型）
     * 标签："storage"（需要在分区表中定义对应的分区）
     */
    recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");

    /**
     * 从Flash加载已注册的人脸ID数据
     * 在系统启动时恢复之前保存的人脸特征库
     * 返回成功加载的人脸ID数量
     */
    int enrolled_num = recognizer->set_ids_from_flash();
    ESP_LOGI(TAG, "从Flash加载了 %d 个人脸ID", enrolled_num);

    /**
     * 用于存储当前帧检测到的人脸关键点坐标
     * 关键点数组包含10个整数（5个关键点 × 2个坐标）
     * 在注册和识别操作时传递给识别器
     */
    std::vector<int> current_landmarks;

    /**
     * 人脸检测状态标志
     * true：当前帧中检测到人脸
     * false：当前帧中未检测到人脸
     */
    bool face_detected = false;

    /**
     * 连续漏检计数：用于"单次漏检不清框、连续2次才清"的显示平滑
     */
    int detect_miss_count = 0;

    /**
     * 上次自动识别的时间戳（陌生人监测的周期控制）
     * 每隔AUTO_RECOGNIZE_INTERVAL_MS毫秒对检测到的人脸自动识别一次
     */
    TickType_t last_auto_rec_tick = 0;

    /**
     * 陌生人警告激活标志
     * true：当前处于陌生人警告状态
     * 用于避免同一陌生人持续在场时日志重复刷屏
     */
    bool stranger_active = false;

    /**
     * 连续识别失败计数（仅相似度 ≥ STRANGER_SIMILARITY_FLOOR 时累加）
     * 达到 STRANGER_CONFIRM_FRAMES 触发报警；人脸消失/识别成功时清零
     */
    int consecutive_stranger_count = 0;

    /**
     * 上次陌生人警告日志输出时间戳
     * stranger_active 期间每隔 STRANGER_LOG_COOLDOWN_MS 输出一次 ESP_LOGW
     */
    TickType_t last_stranger_log_tick = 0;

    /**
     * 上次播报"请注册人脸"提示音的时间戳
     * 人脸库为空时按VOICE_PROMPT_INTERVAL_MS间隔周期播报audio_01
     */
    TickType_t last_voice_prompt_tick = 0;

    /**
     * 上次人脸检测的时间戳
     * 每 DETECT_INTERVAL_MS 做一次完整 MTCNN+MNP01 双阶段检测
     * 间隔内的帧复用 cached_detect_results 画框，不重跑 AI
     */
    TickType_t last_detect_tick = 0;

    /**
     * 缓存最近一次人脸检测的结果
     * 在 3 秒检测间隔内，所有帧都基于此结果画检测框，避免每帧推理
     * 值类型：每次检测后做拷贝，避免 detector 内部引用失效
     */
    std::list<dl::detect::result_t> cached_detect_results;

    // ---- 任务主循环 ----
    while (true)
    {
        /**
         * 第一步：检查是否有按键事件等待处理（非阻塞查看）
         * 使用peek模式，只查看不清除事件标志
         * 后续在帧到达且处理后，再实际消费事件
         * 这样即使没有帧到达，事件也不会丢失
         */
        button_event_t pending_event = peek_button_event();

        // ---- 第二步：处理AI帧数据 ----

        /**
         * 从AI帧队列接收摄像头采集的帧数据
         * 设置100ms超时，避免无限阻塞导致按键响应延迟
         * 接收成功时返回pdTRUE，frame指向接收到的帧缓冲区
         * 超时返回pdFALSE，继续执行后续按键检查
         */
        if (xQueueReceive(xQueueAIFrame, &frame, pdMS_TO_TICKS(100)))
        {
            // 本帧的"全局时间"和"是否执行检测"标志（DETECT_INTERVAL_MS=0 时每帧都检测），供阶段一/阶段二共用
            TickType_t frame_tick = xTaskGetTickCount();
            bool should_run_detect =
                (frame_tick - last_detect_tick) >= pdMS_TO_TICKS(DETECT_INTERVAL_MS);

            // ----- 第三阶段：人脸检测（两级级联，每帧实时执行）-----

            /**
             * 恢复 ESP-WHO 原始每帧检测模式：MTCNN 双阶段推理单帧约 100~300ms，
             * 每帧都跑使画面连续均匀（约5fps）、不会"卡一下"，且每帧都检测
             * 不会漏检、五点持续显示。语音已移至 Core1，AI 独占 Core0，
             * 检测开销不再导致语音卡顿，故无需拉长检测间隔。
             * 间隔内的帧仍复用 cached_detect_results 画框（不重跑 AI）。
             */
            if (should_run_detect)
            {
                last_detect_tick = frame_tick;

                /**
                 * 第一阶段：使用MSR01检测器进行粗检测
                 * 输入参数说明：
                 *   - (uint16_t *)frame->buf：RGB565格式的帧数据缓冲区
                 *   - {(int)frame->height, (int)frame->width, 3}：图像形状
                 *     包含高度、宽度、通道数（RGB为3通道）
                 * 输出：人脸候选区域列表，每个结果包含边界框坐标和关键点
                 */
                std::list<dl::detect::result_t>& detect_candidates = detector.infer(
                    (uint16_t*)frame->buf, {(int)frame->height, (int)frame->width, 3});

                /**
                 * 第二阶段：使用MNP01检测器进行精确检测
                 * 在MSR01生成的候选区域基础上进一步确认和精确定位
                 * 输出更精确的边界框和面部关键点（眼睛、鼻子、嘴角）
                 */
                std::list<dl::detect::result_t>& detect_results =
                    detector2.infer((uint16_t*)frame->buf,
                                    {(int)frame->height, (int)frame->width, 3},
                                    detect_candidates);

                // ----- 更新人脸检测状态 -----

                if (detect_results.size() > 0)
                {
                    // 检测到人脸：更新缓存（值拷贝，detector 内部引用不再依赖）
                    cached_detect_results = detect_results;
                    face_detected = true;
                    detect_miss_count = 0;

                    /**
                     * 保存第一个人脸的关键点数据，用于后续注册/识别
                     * keypoint包含10个int：5个关键点的x/y坐标
                     */
                    current_landmarks = cached_detect_results.front().keypoint;
                }
                else
                {
                    // 未检测到人脸
                    face_detected = false;
                    current_landmarks.clear();

                    // 人脸消失，解除陌生人警告状态并清零计数
                    stranger_active = false;
                    consecutive_stranger_count = 0;

                    // 单次漏检保留上一次的检测框继续显示（避免框一闪一闪、像"识别不到"），
                    // 连续 2 次漏检才真正清除。
                    if (++detect_miss_count >= 2)
                    {
                        cached_detect_results.clear();
                    }
                }

                ESP_LOGD(TAG, "本帧执行检测：检测到 %d 张人脸",
                         (int)cached_detect_results.size());
            }

            /**
             * 画检测框：基于 cached_detect_results，无论本帧是否真正检测都画
             * 这样在 3 秒检测间隔内检测框位置稳定不闪烁
             */
            if (cached_detect_results.size() > 0)
            {
                draw_detection_result(
                    (uint16_t*)frame->buf, frame->height, frame->width,
                    cached_detect_results);
            }

            // ----- 第三步补充：自动识别与语音反馈 -----

            /**
             * 两阶段状态机（配合语音播报，每 5 秒一次检测周期内执行一次）：
             *   阶段一（人脸库为空）：
             *     - 仅当本帧执行了检测（5s 周期到）且检测到人脸 + 距上次播报 ≥ 5s
             *       时播报 audio_01 提示注册；无人在场时完全静默
             *   阶段二（人脸库非空）：
             *     - 每 5 秒对人脸做一次比对（与检测周期对齐）
             *     - 匹配成功（相似度>阈值）：绿色"Welcome" + 播报audio_05
             *     - 匹配失败（相似度<阈值）：红色错误提示 + 播报audio_04
             *       （含相似度门槛 STRANGER_SIMILARITY_FLOOR + 连续帧确认）
             * 注意：
             *   1. 识别必须在帧发送到LCD队列之前完成——帧被LCD任务
             *      显示后会归还给摄像头驱动，此后frame指针失效
             *   2. 阶段一/阶段二均依赖外层 should_run_detect + face_detected 标志
             */
            {
                TickType_t now_tick = xTaskGetTickCount();
                int enrolled_num = recognizer->get_enrolled_id_num();

                // ===== 阶段一：人脸库为空，仅在 3 秒检测到人脸时提示注册 =====
                if (enrolled_num == 0)
                {
                    /**
                     * 触发条件（三个必须同时满足）：
                     *   1. 本帧执行了检测（should_run_detect）：不是检测间隔内的中间帧
                     *   2. 检测到至少一张人脸（face_detected）：避免空场景重复骚扰
                     *   3. 距上次播报已超过 VOICE_PROMPT_INTERVAL_MS(5s)：节流防卡死
                     *
                     * 设计意图：无人在场时完全静默（不播任何音），人进入画面时
                     * 友好提示注册；长占摄像头时最多每 5 秒播一次 audio_01。
                     */
                    if (should_run_detect && face_detected &&
                        (now_tick - last_voice_prompt_tick) >=
                            pdMS_TO_TICKS(VOICE_PROMPT_INTERVAL_MS))
                    {
                        last_voice_prompt_tick = now_tick;
                        voice_play(VOICE_ID_REGISTER_PROMPT); // audio_01：请注册人脸
                        ESP_LOGI(TAG, "人脸库为空且检测到人脸，播报注册提示音");
                    }
                }
                // ===== 阶段二：人脸库非空，每个检测周期(每5秒)自动识别一次 =====
                // 必须同时满足 should_run_detect：识别严格跟随"检测帧"执行，
                // 保证"完成一次识别→等待3秒→再识别"；否则识别与检测是两个
                // 相位错开的3秒定时器，画面会每隔约1.5秒就卡顿一次（看似连续识别）
                else if (should_run_detect && face_detected &&
                         (now_tick - last_auto_rec_tick) >=
                             pdMS_TO_TICKS(AUTO_RECOGNIZE_INTERVAL_MS))
                {
                    // 更新时间戳：3秒内不再识别
                    last_auto_rec_tick = now_tick;

                    /**
                     * 对当前帧中检测到的人脸执行识别
                     * 与已注册的所有人脸特征进行相似度比对
                     */
                    g_recognizing = true; // 通知显示任务：正在识别，叠加转圈动画
                    face_info_t auto_result = recognizer->recognize(
                        (uint16_t*)frame->buf,
                        {(int)frame->height, (int)frame->width, 3},
                        current_landmarks);
                    g_recognizing = false;

                    if (auto_result.id >= 0)
                    {
                        // 匹配到已注册人脸：绿色Welcome欢迎反馈 + 播报识别成功
                        stranger_active = false;
                        consecutive_stranger_count = 0;  // 匹配成功，重置计数
                        set_lcd_welcome("Welcome: %s(%.0f%%)",
                                        auto_result.name.c_str(),
                                        auto_result.similarity * 100.0f);
                        voice_play(VOICE_ID_RECOGNIZE_OK); // audio_05：识别成功
                        ESP_LOGI(TAG, "识别成功！姓名: %s, 相似度: %.2f",
                                 auto_result.name.c_str(), auto_result.similarity);
                    }
                    else
                    {
                        // 识别失败：当前人脸未匹配任何已注册者 → 判定为陌生人候选
                        // 关键修复：不再用 STRANGER_SIMILARITY_FLOOR 过滤低相似度，
                        // 否则真正的陌生人（相似度本就偏低）会被误判为检测器误检而漏报。
                        consecutive_stranger_count++;

                        if (consecutive_stranger_count < STRANGER_CONFIRM_FRAMES)
                        {
                            // 尚未达到连续确认次数，等待下一次（每5秒）判定
                            ESP_LOGD(TAG, "疑似陌生人（相似度 %.0f%%），"
                                          "等待连续确认 %d/%d",
                                     auto_result.similarity * 100.0f,
                                     consecutive_stranger_count,
                                     STRANGER_CONFIRM_FRAMES);
                        }
                        else
                        {
                            /**
                             * 连续 N 次均识别失败 → 确认为陌生人，触发报警。
                             * stranger_active 抑制同一陌生人在场期间的重复报警。
                             */
                            consecutive_stranger_count++;
                            set_lcd_warn("错误: 陌生人!(%.0f%%)",
                                         auto_result.similarity * 100.0f);

                            // 仅在状态切换时（首次确认）播放 audio_04 警告音
                            if (!stranger_active)
                            {
                                stranger_active = true;
                                last_stranger_log_tick = now_tick;
                                voice_play(VOICE_ID_RECOGNIZE_FAIL); // audio_04：陌生人告警
                                ESP_LOGW(TAG, "警告：检测到陌生人！"
                                              "最高相似度: %.2f (阈值: %.2f)",
                                         auto_result.similarity, recognizer->get_thresh());
                            }
                            else
                            {
                                // 同一陌生人在场期间：节流日志（避免 ESP_LOGW 刷屏）
                                if ((now_tick - last_stranger_log_tick) >=
                                    pdMS_TO_TICKS(STRANGER_LOG_COOLDOWN_MS))
                                {
                                    last_stranger_log_tick = now_tick;
                                    ESP_LOGW(TAG, "陌生人持续在场中... 相似度: %.2f",
                                             auto_result.similarity);
                                }
                            }
                        }
                    }
                }
            }

            // ----- 第四步：发送帧到LCD显示 -----

            /**
             * 优先将帧发送到LCD显示队列
             * 此操作必须放在Flash写入之前，因为LCD与Flash共用SPI总线
             * 如果先写入Flash再发送LCD，SPI总线被Flash占用导致LCD显示失败（黑屏）
             */
            if (xQueueLCDFrame)
            {
                /**
                 * LCD队列存在，将帧指针发送到队列
                 * LCD显示任务从此队列接收帧并绘制到屏幕
                 * portMAX_DELAY：如果队列满则无限等待
                 */
                xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
            }
            else
            {
                /**
                 * LCD队列不存在，将帧返回给摄像头驱动复用
                 * 注意：返回后frame指针变为无效，需设置为NULL
                 */
                esp_camera_fb_return(frame);
                frame = NULL;
            }
        }

        // ---- 第五步：处理按键事件 ----

        /**
         * 独立处理按键事件（无论是否有帧到达）
         * 使用peek模式查看的事件，在此处统一消费处理
         * 确保所有按键事件都被及时响应
         */
        if (pending_event != BUTTON_EVENT_NONE)
        {
            /**
             * 消费按键事件（清除标志，防止重复处理）
             * 注意：必须调用get_button_event()清除事件标志
             * 否则事件会一直挂起，导致下次循环重复触发
             */
            get_button_event();

            // ==================== 双击事件处理 ====================
            /**
             * 双击事件：清空所有人脸数据
             * 此操作不需要摄像头帧数据，任何状态下双击都立即执行
             * 流程：
             *   1. 获取所有已注册的人脸ID列表
             *   2. 逐个删除每个人脸ID（不更新Flash）
             *   3. 最后统一写入Flash更新
             */
            if (pending_event == BUTTON_EVENT_DOUBLE_CLICK)
            {
                ESP_LOGI(TAG, "执行清空所有人脸数据操作...");
                set_lcd_info("清空数据");  // LCD显示清空操作信息

                // 获取所有已注册的人脸ID信息列表
                std::vector<face_info_t> ids = recognizer->get_enrolled_ids();

                // 遍历所有人脸ID，逐个删除（不立即写入Flash）
                for (auto& info : ids)
                {
                    recognizer->delete_id(info.id, false);
                }

                // 统一将删除结果写入Flash持久化
                recognizer->write_ids_to_flash();
                ESP_LOGI(TAG, "已清空 %d 个人脸数据并更新Flash", (int)ids.size());

                // 使用continue跳过后续的帧数据检查，直接进入下一轮循环
                continue;
            }

            // ==================== 单击/长按事件检查 ====================
            /**
             * 单击和长按事件需要有效的摄像头帧数据才能执行
             * 检查条件：
             *   1. face_detected == true：当前帧检测到人脸
             *   2. frame != NULL：有有效的帧缓冲区（未释放）
             * 如果条件不满足，打印警告并跳过事件处理
             */

            // 检查帧缓冲区是否有效（单击识别会当场自行检测，故不要求已检测到人脸）
            if (frame == NULL)
            {
                ESP_LOGW(TAG, "检测到按键事件(%d)，但无有效的帧数据",
                         pending_event);
                set_lcd_info("无帧数据");  // LCD提示无帧数据
            }
            // 长按(注册)需要当前已检测到人脸；单击识别则不需要
            else if (pending_event != BUTTON_EVENT_SINGLE_CLICK && !face_detected)
            {
                ESP_LOGW(TAG, "检测到按键事件(%d)，但未检测到人脸，请对准摄像头后重试",
                         pending_event);
                set_lcd_info("请对准人脸");  // LCD提示对准人脸
            }
            // 条件满足，执行具体的按键事件处理
            else
            {
                // 根据事件类型执行不同的操作
                switch (pending_event)
                {

                    // ========== 长按事件：注册人脸 ==========
                    case BUTTON_EVENT_LONG_PRESS:
                    {
                        ESP_LOGI(TAG, "执行人脸注册操作...");

                        /**
                         * 生成默认人脸名称
                         * 格式：Person_n（n为当前已注册的人脸数量）
                         * 例如：Person_0, Person_1, ...
                         * 使用snprintf确保缓冲区不溢出
                         */
                        char name[32];
                        snprintf(name, sizeof(name), "Person_%d",
                                 recognizer->get_enrolled_id_num());

                        /**
                         * 执行人脸注册（录入）
                         * 参数说明：
                         *   - frame->buf：摄像头帧数据（RGB565格式）
                         *   - 图像形状：{高, 宽, 3}（3通道RGB）
                         *   - current_landmarks：人脸关键点坐标
                         *   - name：人脸名称
                         *   - true：同步写入Flash
                         * 返回值：>=0 注册成功（返回人脸ID索引），<0 注册失败
                         */
                        g_recognizing = true; // 通知显示任务：正在注册，叠加转圈动画
                        int id = recognizer->enroll_id(
                            (uint16_t*)frame->buf,
                            {(int)frame->height, (int)frame->width, 3},
                            current_landmarks, name, true);
                        g_recognizing = false;

                        // 检查注册结果
                        if (id >= 0)
                        {
                            // 注册成功，打印ID和名称
                            ESP_LOGI(TAG, "人脸注册成功！ID: %d, 名称: %s", id, name);
                            set_lcd_info("注册: %s", name);  // LCD显示注册成功
                            voice_play(VOICE_ID_ENROLL_OK);  // audio_02：注册成功，进入识别阶段

                            /**
                             * 将注册的人脸数据写入Flash分区
                             * 确保系统重启后数据不丢失
                             * 返回成功写入的人脸ID数量
                             */
                            int written = recognizer->write_ids_to_flash();
                            ESP_LOGI(TAG, "已写入 %d 个人脸数据到Flash", written);
                        }
                        else
                        {
                            // 注册失败，常见原因：人脸未对齐、特征提取失败
                            ESP_LOGE(TAG, "人脸注册失败！");
                            set_lcd_info("注册失败");            // LCD显示注册失败
                            voice_play(VOICE_ID_ENROLL_FAIL);    // audio_03：注册失败，请重新注册
                        }
                        break;
                    }

                    // ========== 单击事件：手动识别（当场检测 + 识别） ==========
                    case BUTTON_EVENT_SINGLE_CLICK:
                    {
                        // 人脸库为空时不做识别反馈，提示先注册（语音播报注册提示）
                        if (recognizer->get_enrolled_id_num() == 0)
                        {
                            ESP_LOGW(TAG, "人脸库为空，请先长按注册人脸");
                            set_lcd_info("请先注册人脸");
                            voice_play(VOICE_ID_REGISTER_PROMPT); // audio_01：请注册人脸
                            break;
                        }

                        ESP_LOGI(TAG, "手动识别：对当前帧执行检测+识别...");

                        /**
                         * 手动识别不受周期检测限制：当场对当前帧做一次
                         * MSR01+MNP01 检测，拿到最新关键点后再识别。
                         * 这样即使周期检测刚好漏检，单击也能正常识别。
                         */
                        std::list<dl::detect::result_t>& manual_candidates = detector.infer(
                            (uint16_t*)frame->buf, {(int)frame->height, (int)frame->width, 3});
                        std::list<dl::detect::result_t>& manual_faces = detector2.infer(
                            (uint16_t*)frame->buf,
                            {(int)frame->height, (int)frame->width, 3},
                            manual_candidates);

                        if (manual_faces.empty())
                        {
                            ESP_LOGW(TAG, "手动识别：当前帧未检测到人脸");
                            set_lcd_info("未检测到人脸，请对准摄像头");
                            break;
                        }

                        // 更新检测缓存，让下一帧起画面显示检测框/五点
                        cached_detect_results = manual_faces;

                        // 执行识别（使用刚检测到的关键点）
                        g_recognizing = true; // 通知显示任务：正在识别，叠加转圈动画
                        face_info_t result = recognizer->recognize(
                            (uint16_t*)frame->buf,
                            {(int)frame->height, (int)frame->width, 3},
                            cached_detect_results.front().keypoint);
                        g_recognizing = false;

                        // 检查识别结果
                        if (result.id >= 0)
                        {
                            display_recognition_result(result.name.c_str(), result.similarity, 10, 10);
                            set_lcd_welcome("Welcome: %s(%.0f%%)",
                                            result.name.c_str(),
                                            result.similarity * 100.0f);
                            voice_play(VOICE_ID_RECOGNIZE_OK); // audio_05：识别成功
                            ESP_LOGI(TAG, "识别结果 - ID: %d, 姓名: %s, 相似度: %.2f",
                                     result.id, result.name.c_str(), result.similarity);
                        }
                        else
                        {
                            ESP_LOGW(TAG, "未识别到匹配的人脸，最高相似度: %.2f (阈值: %.2f)",
                                     result.similarity, recognizer->get_thresh());
                            set_lcd_warn("错误: 陌生人!(%.0f%%)", result.similarity * 100.0f);
                            voice_play(VOICE_ID_RECOGNIZE_FAIL); // audio_04：识别失败
                        }
                        break;
                    }

                    // 默认情况（不应发生）
                    default:
                        break;
                }
            }
        }
    }
}

// =================================================================
// 识别"转圈等待"动画（手绘，不依赖 LVGL，使用现有 LCD 原语）
// =================================================================

/** 转圈动画几何参数（像素）：半径 / 点半径 / 圆周格数 / 弧长格数 */
#define SPINNER_R     16
#define SPINNER_DOT   1
#define SPINNER_SLOTS 16  // 圆周分成 16 格
#define SPINNER_ARC   12  // 显示 12 格(270°)，其余为缺口
#define SPINNER_BOX   (2 * (SPINNER_R + SPINNER_DOT + 3))
/** 显示任务队列等待超时（毫秒）：越小转圈越顺滑 */
#define SPINNER_TICK_MS 40

/** 视频窗口装饰边框：线宽与颜色（用现有原语绘制，不依赖 LVGL） */
#define FRAME_W      6        // 主色带宽度（像素）—— 加宽后的边框
#define FRAME_COLOR  0x2E9F   // 边框主色（青蓝）
#define FRAME_LIGHT  0xE7FF   // 内沿亮线（近白微青，增加层次）
#define FRAME_DARK   0x18E3   // 转圈描边/深色（保留）

/**
 * @brief 在视频画面正中央绘制一帧"彗星弧"转圈动画
 *
 * 识别推理会阻塞 AI 任务，期间画面暂时冻结。本函数在冻结画面上叠加一个
 * 旋转的弧段（一端亮、一端暗，并带缺口），让"卡顿"看起来是在等待处理。
 * 只画弧段本身（含深色描边保证任何画面下都清晰），不铺任何整块/圆环底盘，
 * 因此不会出现"横条/突兀色块"。先还原该小块背景再叠加，最后一次性刷到屏幕。
 *
 * @param bg   当前缩放后的视频缓冲（VIDEO_DISPLAY_W × VIDEO_DISPLAY_H，行宽 VIDEO_DISPLAY_W）
 * @param step 旋转步号（调用者每次递增）
 */
static void draw_recognition_spinner(const uint16_t* bg, int step)
{
    // 区域位于视频窗口正中央（缓冲内坐标）
    const int bx = VIDEO_DISPLAY_W / 2 - SPINNER_BOX / 2; // 缓冲内左上角 X
    const int by = VIDEO_DISPLAY_H / 2 - SPINNER_BOX / 2; // 缓冲内左上角 Y
    const int cc = SPINNER_BOX / 2;                       // 圆心（缓冲内）

    // 16 个方向的单位向量（×1000），避免运行时三角函数
    static const int s_cos[16] = {1000, 924, 707, 383, 0, -383, -707, -924,
                                  -1000, -924, -707, -383, 0, 383, 707, 924};
    static const int s_sin[16] = {0, 383, 707, 924, 1000, 924, 707, 383,
                                  0, -383, -707, -924, -1000, -924, -707, -383};
    // 弧段由亮到暗的渐变（RGB565）
    static const uint16_t s_arc_col[SPINNER_ARC] = {
        0xFFFF, 0xE73C, 0xD6BA, 0xC638, 0xB5B6, 0xA534,
        0x94B2, 0x8430, 0x73AE, 0x632C, 0x52AA, 0x4248};

    static uint16_t s_buf[SPINNER_BOX * SPINNER_BOX];

    // 1. 还原该小块背景
    for (int yy = 0; yy < SPINNER_BOX; yy++)
    {
        memcpy(s_buf + yy * SPINNER_BOX,
               bg + (by + yy) * VIDEO_DISPLAY_W + bx,
               SPINNER_BOX * sizeof(uint16_t));
    }

    // 2. 绘制弧段：每点先铺 5x5 深色描边，再叠 3x3 亮色，保证任何画面下都清晰
    int head = ((step % SPINNER_SLOTS) + SPINNER_SLOTS) % SPINNER_SLOTS;
    for (int t = 0; t < SPINNER_ARC; t++)
    {
        int i = ((head - t) % SPINNER_SLOTS + SPINNER_SLOTS) % SPINNER_SLOTS;
        int px = cc + (SPINNER_R * s_cos[i]) / 1000;
        int py = cc + (SPINNER_R * s_sin[i]) / 1000;

        for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++)
            {
                int X = px + dx, Y = py + dy;
                if (X >= 0 && X < SPINNER_BOX && Y >= 0 && Y < SPINNER_BOX)
                    s_buf[Y * SPINNER_BOX + X] = FRAME_DARK;
            }
        for (int dy = -SPINNER_DOT; dy <= SPINNER_DOT; dy++)
            for (int dx = -SPINNER_DOT; dx <= SPINNER_DOT; dx++)
            {
                int X = px + dx, Y = py + dy;
                if (X >= 0 && X < SPINNER_BOX && Y >= 0 && Y < SPINNER_BOX)
                    s_buf[Y * SPINNER_BOX + X] = s_arc_col[t];
            }
    }

    // 3. 一次性刷到屏幕（叠加视频窗口在屏幕上的偏移）
    lcd_draw_bitmap(VIDEO_DISPLAY_X + bx, VIDEO_DISPLAY_Y + by,
                    VIDEO_DISPLAY_X + bx + SPINNER_BOX,
                    VIDEO_DISPLAY_Y + by + SPINNER_BOX, s_buf);
}

/**
 * @brief 在视频窗口四周绘制装饰边框（不依赖 LVGL，用现有填充原语）
 *
 * 样式：一圈加宽的青蓝主色带（FRAME_W px）+ 紧贴视频的内沿亮线（1px），
 * 形成"金属倒角高光"的层次感，比单色描边更精致。
 * 边框位于视频窗口之外，视频刷新不会覆盖它，故只需绘制一次。
 */
static void draw_video_frame(void)
{
    const int x0 = VIDEO_DISPLAY_X, y0 = VIDEO_DISPLAY_Y;
    const int w = VIDEO_DISPLAY_W, h = VIDEO_DISPLAY_H;
    const int x1 = x0 + w, y1 = y0 + h;

    // 1) 主色带（FRAME_W px，位于内亮线之外）
    lcd_fill_rect(x0 - 1 - FRAME_W, y0 - 1 - FRAME_W,
                  w + 2 + 2 * FRAME_W, FRAME_W, FRAME_COLOR); // 上
    lcd_fill_rect(x0 - 1 - FRAME_W, y1 + 1,
                  w + 2 + 2 * FRAME_W, FRAME_W, FRAME_COLOR); // 下
    lcd_fill_rect(x0 - 1 - FRAME_W, y0 - 1 - FRAME_W,
                  FRAME_W, h + 2 + 2 * FRAME_W, FRAME_COLOR); // 左
    lcd_fill_rect(x1 + 1, y0 - 1 - FRAME_W,
                  FRAME_W, h + 2 + 2 * FRAME_W, FRAME_COLOR); // 右

    // 2) 内沿亮线（1px，紧贴视频）—— 形成高光层次
    lcd_fill_rect(x0 - 1, y0 - 1, w + 2, 1, FRAME_LIGHT); // 上
    lcd_fill_rect(x0 - 1, y1, w + 2, 1, FRAME_LIGHT);     // 下
    lcd_fill_rect(x0 - 1, y0 - 1, 1, h + 2, FRAME_LIGHT); // 左
    lcd_fill_rect(x1, y0 - 1, 1, h + 2, FRAME_LIGHT);     // 右
}

// =================================================================
// LCD图像显示任务（task_process_lcd）
// =================================================================

/**
 * @brief LCD图像显示任务
 *
 * 该任务从LCD帧队列接收已处理的帧数据，通过SPI接口
 * 将图像绘制到LCD屏幕上。显示完成后将帧缓冲区返回
 * 给摄像头驱动以供复用。
 *
 * @param arg 任务参数指针（本实现中未使用，传入NULL）
 * @return 无返回值（任务循环运行，永不返回）
 *
 * @note 任务栈大小：4KB
 * @note 优先级：5
 * @note 运行核心：0（应用核心）
 * @note 使用portMAX_DELAY无限等待队列数据
 * @note 显示完成后必须调用esp_camera_fb_return()释放帧缓冲区
 */
static void task_process_lcd(void* arg)
{
    // 声明帧缓冲区指针，用于接收AI处理后的帧数据
    camera_fb_t* frame = NULL;

    /**
     * 分配视频窗口缓冲（PSRAM）
     * 摄像头QVGA(320x240)帧存放到居中视频窗口 320x240 后存放于此
     */
    uint16_t* scale_buf = (uint16_t*)heap_caps_malloc(
        VIDEO_DISPLAY_W * VIDEO_DISPLAY_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!scale_buf)
    {
        // PSRAM分配失败时退回内部RAM
        scale_buf = (uint16_t*)heap_caps_malloc(
            VIDEO_DISPLAY_W * VIDEO_DISPLAY_H * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }
    if (!scale_buf)
    {
        ESP_LOGE(TAG, "缩放缓冲区分配失败，LCD显示任务退出");
        vTaskDelete(NULL);
        return;
    }

    // 任务主循环，持续处理LCD显示请求
    // 说明：采用带超时的队列等待——无新帧时若正在识别，则叠加"转圈等待"动画
    bool has_frame = false;       // 是否已绘制过至少一帧（转圈还原背景需要）
    int  spin_step = 0;           // 转圈动画步号
    char prev_info_text[64] = ""; // 上一帧信息条文本（避免重复重绘）
    uint16_t prev_info_fc = WHITE;
    uint16_t prev_info_bc = BLACK;

    while (true)
    {
        // ============ 1. 接收一帧（带超时，便于识别卡顿期间刷新转圈动画） ============
        if (xQueueReceive(xQueueLCDFrame, &frame, pdMS_TO_TICKS(SPINNER_TICK_MS)))
        {
            // ============ 2. 读取信息条状态 ============
            bool info_active = false;
            char info_text[64] = "";
            uint16_t info_fc = WHITE, info_bc = BLACK;
            if (lcd_info_mutex && xSemaphoreTake(lcd_info_mutex, portMAX_DELAY) == pdTRUE)
            {
                // 超时则置为非激活
                if (g_lcd_info.active &&
                    (xTaskGetTickCount() - g_lcd_info.timestamp) >= pdMS_TO_TICKS(INFO_CLEAR_MS))
                {
                    g_lcd_info.active = false;
                }
                info_active = g_lcd_info.active;
                if (info_active)
                {
                    strncpy(info_text, g_lcd_info.text, sizeof(info_text) - 1);
                    info_text[sizeof(info_text) - 1] = '\0';
                    info_fc = g_lcd_info.fc;
                    info_bc = g_lcd_info.bc;
                }
                xSemaphoreGive(lcd_info_mutex);
            }

            // ============ 3. 缩放帧到视频窗口缓冲（320x240，1:1） ============
            /**
             * 将摄像头 QVGA(320x240) 帧绘制到居中窗口（尺寸一致时为 1:1）。
             * 检测框已由 AI 任务画在 frame->buf 上，随画面一起显示。
             */
            for (int y = 0; y < VIDEO_DISPLAY_H; y++)
            {
                const uint16_t* src = (const uint16_t*)frame->buf +
                                      (y * frame->height / VIDEO_DISPLAY_H) * frame->width;
                uint16_t* dst = scale_buf + y * VIDEO_DISPLAY_W;
                for (int x = 0; x < VIDEO_DISPLAY_W; x++)
                {
                    dst[x] = src[x * frame->width / VIDEO_DISPLAY_W];
                }
            }

            // ============ 4. 分块绘制视频窗口（每块 160 行） ============
            for (int y = 0; y < VIDEO_DISPLAY_H; y += 160)
            {
                int rows = (y + 160 > VIDEO_DISPLAY_H) ? VIDEO_DISPLAY_H - y : 160; // 末块不足 160 行时截断
                lcd_draw_bitmap(VIDEO_DISPLAY_X, VIDEO_DISPLAY_Y + y,
                                VIDEO_DISPLAY_X + VIDEO_DISPLAY_W,
                                VIDEO_DISPLAY_Y + y + rows,
                                scale_buf + y * VIDEO_DISPLAY_W);
            }

            // 归还帧缓冲给摄像头驱动
            esp_camera_fb_return(frame);
            has_frame = true;

            // 首帧绘制后，绘制一次视频窗口装饰边框（位于视频窗口外，不会被覆盖）
            static bool frame_border_drawn = false;
            if (!frame_border_drawn)
            {
                draw_video_frame();
                frame_border_drawn = true;
            }

            // ============ 5. 绘制底部信息条 ============
            // 视频窗口不覆盖信息栏：内容变化时重绘一次；消失时清黑一次。
            if (info_active)
            {
                if (strcmp(prev_info_text, info_text) != 0 ||
                    prev_info_fc != info_fc || prev_info_bc != info_bc)
                {
                    strncpy(prev_info_text, info_text, sizeof(prev_info_text) - 1);
                    prev_info_text[sizeof(prev_info_text) - 1] = '\0';
                    prev_info_fc = info_fc;
                    prev_info_bc = info_bc;
                    lcd_fill_rect(LCD_INFO_X, LCD_INFO_Y, LCD_INFO_W, LCD_INFO_H, info_bc);
                    lcd_display_string(LCD_INFO_X, LCD_INFO_Y + LCD_INFO_TEXT_Y_OFFSET,
                                       info_fc, info_bc, LCD_INFO_FONT, info_text);
                }
            }
            else if (prev_info_text[0] != '\0')
            {
                // 信息条消失：清为黑底，避免残留（视频窗口覆盖不到该区域）
                lcd_fill_rect(LCD_INFO_X, LCD_INFO_Y, LCD_INFO_W, LCD_INFO_H, BLACK);
                prev_info_text[0] = '\0';
            }
        }

        // ============ 6. 识别卡顿期间：叠加"转圈等待"动画 ============
        // 推理在 Core0、本任务在 Core1，动画不会被推理阻塞；用动画把停顿
        // 表现成"正在处理"，使卡顿显得合理。
        if (g_recognizing && has_frame)
        {
            draw_recognition_spinner(scale_buf, spin_step);
            spin_step = (spin_step + 1) % 8;
        }
    }
}

// =================================================================
// 摄像头图像采集任务（task_process_camera）
// =================================================================

/**
 * @brief 摄像头图像采集任务
 *
 * 该任务循环从摄像头驱动获取一帧图像数据，然后将帧指针
 * 发送到AI处理队列供检测任务处理。
 * 摄像头的帧率由硬件自动控制，本任务仅负责搬运数据。
 *
 * @param arg 任务参数指针（本实现中未使用，传入NULL）
 * @return 无返回值（任务循环运行，永不返回）
 *
 * @note 任务栈大小：3KB
 * @note 优先级：5
 * @note 运行核心：1（协议核心，与CPU密集型AI任务分离）
 * @note 如果AI队列满，xQueueSend会阻塞等待
 * @note 帧缓冲区由摄像头驱动管理，任务不需要关心分配和释放
 */
static void task_process_camera(void* arg)
{
    // 任务主循环，持续采集摄像头帧数据
    while (true)
    {
        /**
         * 从摄像头驱动获取一帧图像数据
         * 返回指向帧缓冲区结构体的指针，包含：
         *   - buf：图像数据缓冲区
         *   - len：数据长度
         *   - width：图像宽度（像素）
         *   - height：图像高度（像素）
         *   - format：像素格式（RGB565/JPEG等）
         * 如果获取失败（摄像头未初始化或硬件错误）返回NULL
         */
        camera_fb_t* frame = esp_camera_fb_get();

        // 检查帧数据是否获取成功
        if (frame)
        {
            /**
             * 将帧指针发送到AI处理队列
             * AI检测任务从该队列接收帧并进行人脸检测和识别
             * portMAX_DELAY表示如果队列满则无限等待
             * 队列深度为2，摄像头帧率会自动适应AI处理速度
             */
            xQueueSend(xQueueAIFrame, &frame, portMAX_DELAY);
        }
    }
}

// =================================================================
// 应用初始化函数（app_human_face_recognition）
// =================================================================

/**
 * @brief 初始化并启动人脸识别应用
 *
 * 该函数创建所有人脸识别系统所需的同步原语和FreeRTOS任务。
 * 调用该函数前必须先完成摄像头和LCD的硬件初始化。
 *
 * 初始化顺序：
 * 1. 创建按键事件互斥锁 → 失败则退出
 * 2. 创建LCD帧队列（长度2）→ 用于AI→LCD的数据传递
 * 3. 创建AI帧队列（长度2）→ 用于摄像头→AI的数据传递
 * 4. 创建摄像头采集任务 → 绑定到核心1（协议核心）
 * 5. 创建LCD显示任务 → 绑定到核心0（应用核心）
 * 6. 创建AI处理任务 → 绑定到核心0，栈8KB
 * 7. 创建按键检测任务 → 绑定到核心0，栈4KB
 *
 * @return 无返回值
 *
 * @note 互斥锁创建失败时直接返回，不启动任何任务
 * @note 摄像头初始化应在调用此函数之前完成
 * @note LCD初始化应在调用此函数之前完成
 * @note 队列深度为2，适合3级流水线（采集→处理→显示）工作模式
 * @note 核心分配策略：
 *   - 核心1：摄像头采集（I/O密集型）
 *   - 核心0：AI计算、LCD显示、按键检测（CPU密集型）
 */
void app_human_face_recognition(void)
{
    // ---- 1. 创建按键互斥锁 ----

    /**
     * 创建互斥锁，用于保护按键事件全局变量的并发访问
     * 互斥锁相比二值信号量具有优先级继承特性，
     * 可防止优先级反转问题
     */
    button_mutex = xSemaphoreCreateMutex();

    // 检查互斥锁是否创建成功
    if (button_mutex == NULL)
    {
        /**
         * 互斥锁创建失败，系统资源不足
         * 打印错误日志并退出初始化
         * 此时系统无法保证按键事件的线程安全
         */
        ESP_LOGE(TAG, "创建按键互斥锁失败！");
        return;
    }

    // ---- 1b. 创建LCD信息互斥锁 ----

    /**
     * 创建互斥锁，用于保护LCD关键信息显示变量的并发访问
     * 在task_process_ai（写）和task_process_lcd（读/写）之间提供同步
     */
    lcd_info_mutex = xSemaphoreCreateMutex();

    // 检查互斥锁是否创建成功
    if (lcd_info_mutex == NULL)
    {
        ESP_LOGE(TAG, "创建LCD信息互斥锁失败！");
        return;
    }

    // ---- 1c. 初始化语音播报模块 ----

    /**
     * 初始化语音播报（I2S + ES8311 + 播报任务）
     * 用于：空库提示注册(audio_01)、注册成功(audio_02)、注册失败(audio_03)、
     *       识别失败(audio_04)、识别成功(audio_05)
     * 注意：依赖 main.c 中已完成的 i2c_init()
     */
    if (voice_player_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "语音播报初始化失败（不影响人脸识别主流程）");
    }

    // ---- 2. 创建消息队列 ----

    /**
     * 创建LCD帧队列
     * 队列长度：2个元素
     * 元素大小：sizeof(camera_fb_t *) = 4字节（指针）
     * 作用：AI处理任务 → LCD显示任务 的数据传递通道
     * 深度2可以缓冲1帧，应对短暂的显示延迟
     */
    xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t*));

    /**
     * 创建AI帧队列
     * 队列长度：2个元素
     * 元素大小：sizeof(camera_fb_t *) = 4字节（指针）
     * 作用：摄像头采集任务 → AI处理任务 的数据传递通道
     * 深度2可以缓冲1帧，平滑处理帧率波动
     */
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t*));

    // ---- 3. 创建FreeRTOS任务 ----

    /**
     * 创建摄像头采集任务
     * 任务名：     "task_process_camera"
     * 函数：      task_process_camera
     * 栈大小：    3KB（足够用于帧获取和队列发送）
     * 优先级：    5（较高优先级，确保帧数据及时采集）
     * 绑定核心：  1（协议核心，与CPU密集型任务分离）
     * 任务参数：  NULL
     * 任务句柄：  NULL（无需挂起任务）
     */
    xTaskCreatePinnedToCore(task_process_camera, "task_process_camera", 3 * 1024, NULL, 5, NULL, 1);

    /**
     * 创建LCD显示任务
     * 任务名：     "task_process_lcd"
     * 函数：      task_process_lcd
     * 栈大小：    4KB（足够用于SPI数据传输）
     * 优先级：    5（与AI任务同级，确保显示流畅）
     * 绑定核心：  0（应用核心）
     * 任务句柄：  NULL（无需挂起任务）
     */
    xTaskCreatePinnedToCore(task_process_lcd, "task_process_lcd", 4 * 1024, NULL, 5, NULL, 1);

    /**
     * 创建AI人脸检测与识别任务
     * 任务名：     "task_process_ai"
     * 函数：      task_process_ai
     * 栈大小：    8KB（满足神经网络推理与图像处理需求）
     * 优先级：    5（核心任务，最高优先级组）
     * 绑定核心：  0（应用核心，与摄像头采集任务分离）
     * 任务句柄：  NULL
     */
    xTaskCreatePinnedToCore(task_process_ai, "task_process_ai", 8 * 1024, NULL, 5, NULL, 0);

    /**
     * 创建按键检测任务
     * 任务名：     "button_task"
     * 函数：      button_task
     * 栈大小：    4KB（从原2KB增加，解决栈溢出问题）
     * 优先级：    3（较低优先级，按键检测对实时性要求不高）
     * 绑定核心：  0（应用核心）
     * 说明：      原2KB栈在ESP_LOGI调用vsnprintf时溢出
     */
    xTaskCreatePinnedToCore(button_task, "button_task", 4 * 1024, NULL, 3, NULL, 0);

    // 打印应用启动完成日志
    ESP_LOGI(TAG, "人脸识别应用已启动");
}
