/**
 * @file voice_player.c
 * @brief 语音播报模块源文件
 *
 * 功能说明：
 *   将 main/audio/ 目录下内嵌的 5 段 PCM 提示音（audio_01.pcm ~ audio_05.pcm）
 *   通过 ES8311 + I2S 播放，用于人脸识别流程的语音反馈。
 *
 * 实现方式：
 *   1. 音频数据通过 main/CMakeLists.txt 的 EMBED_FILES 链接进固件，
 *      编译后生成 _binary_audio_0X_pcm_start / _end 符号
 *      （ESP-IDF 只取文件名，不含子目录 audio/）；
 *   2. voice_player_init() 依次完成：I2S 初始化、ES8311 初始化、
 *      拉高 PCA9557 IO1 (PA_EN) 启用板载扬声器功放、创建播报队列与任务；
 *   3. voice_play() 仅投递请求（非阻塞），播报任务取出后阻塞写入 I2S；
 *   4. 播报前清空队列中的陈旧请求，保证语音始终与当前状态一致。
 *
 * 音频格式（必须与 i2s.h 配置一致）：16kHz / 16bit / 双声道 / 原始 PCM。
 */

#include "voice_player.h"
#include "es8311_codec.h"
#include "i2s.h"
#include "pca9557.h"         // 控制板载扬声器功放使能引脚（PA_EN = IO1）
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stddef.h>

/** 日志标签 */
static const char *TAG = "voice_player";

/* ==================== 内嵌 PCM 音频数据符号 ==================== */
/**
 * 由 CMakeLists.txt 的 EMBED_FILES 生成。
 * 注意：ESP-IDF 生成符号时只取"文件名"（不含子目录 audio/），
 * 因此 audio/audio_01.pcm 对应的符号为 _binary_audio_01_pcm_start/_end。
 */
extern const uint8_t audio_01_pcm_start[] asm("_binary_audio_01_pcm_start");
extern const uint8_t audio_01_pcm_end[] asm("_binary_audio_01_pcm_end");
extern const uint8_t audio_02_pcm_start[] asm("_binary_audio_02_pcm_start");
extern const uint8_t audio_02_pcm_end[] asm("_binary_audio_02_pcm_end");
extern const uint8_t audio_03_pcm_start[] asm("_binary_audio_03_pcm_start");
extern const uint8_t audio_03_pcm_end[] asm("_binary_audio_03_pcm_end");
extern const uint8_t audio_04_pcm_start[] asm("_binary_audio_04_pcm_start");
extern const uint8_t audio_04_pcm_end[] asm("_binary_audio_04_pcm_end");
extern const uint8_t audio_05_pcm_start[] asm("_binary_audio_05_pcm_start");
extern const uint8_t audio_05_pcm_end[] asm("_binary_audio_05_pcm_end");

/**
 * @struct voice_clip_t
 * @brief 一段内嵌音频的起止地址
 */
typedef struct
{
    const uint8_t *start; /**< 数据起始地址 */
    const uint8_t *end;   /**< 数据结束地址（不含） */
} voice_clip_t;

/** 语音片段表：下标与 voice_id_t 一一对应 */
static const voice_clip_t voice_clips[VOICE_ID_MAX] = {
    {audio_01_pcm_start, audio_01_pcm_end}, /* 01 提示注册人脸 */
    {audio_02_pcm_start, audio_02_pcm_end}, /* 02 注册成功 */
    {audio_03_pcm_start, audio_03_pcm_end}, /* 03 注册失败 */
    {audio_04_pcm_start, audio_04_pcm_end}, /* 04 识别失败 */
    {audio_05_pcm_start, audio_05_pcm_end}, /* 05 识别成功 */
};

/** 播报请求队列（元素为 voice_id_t） */
static QueueHandle_t voice_queue = NULL;

/**
 * @brief 播报占用标志
 *
 * true：当前正在播报某段音频，期间 voice_play() 投递的请求被丢弃
 * false：空闲，可接受新请求
 *
 * 设计原因：一段音频播完需要数百毫秒到几秒。如果在播报中再次
 * voice_play()，I2S 写入队列会塞入半截数据导致欠喂/卡死。
 * 用此标志实现"播报期间拒收新请求"，配合"3 秒一次检测+识别"
 * 节奏即可保证一段播完后再触发下一段。
 */
static volatile bool voice_playing = false;

/**
 * @brief 播报任务：从队列取请求并通过 I2S 播放整段音频
 * @param arg 任务参数（未使用）
 * @return 无返回值（任务循环运行）
 */
static void voice_task(void *arg)
{
    voice_id_t id = VOICE_ID_REGISTER_PROMPT;

    while (true)
    {
        // 阻塞等待播报请求
        if (xQueueReceive(voice_queue, &id, portMAX_DELAY) == pdTRUE)
        {
            if (id >= VOICE_ID_MAX)
                continue;

            // 丢弃队列中积压的陈旧请求，保证播报的是最新状态
            xQueueReset(voice_queue);

            const voice_clip_t *clip = &voice_clips[id];
            size_t total = (size_t)(clip->end - clip->start);
            if (total == 0)
                continue;

            // 标记为"正在播报"，期间 voice_play() 投递的请求会被丢弃
            voice_playing = true;

            ESP_LOGI(TAG, "播放语音 %02d (%u 字节)", (int)id + 1, (unsigned)total);

            // 阻塞写入整段 PCM 数据
            esp_err_t ret = i2s_write_pcm(clip->start, total);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "语音 %02d 播放失败: %d", (int)id + 1, (int)ret);
            }

            // 播完释放标志
            voice_playing = false;
        }
    }
}

/**
 * @brief 初始化语音播报模块
 */
esp_err_t voice_player_init(void)
{
    // 1. 初始化 I2S 发送通道（16kHz/16bit/立体声）
    esp_err_t ret = i2s_driver_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_driver_init failed: %d", (int)ret);
        return ret;
    }

    // 2. 初始化 ES8311 编解码芯片（依赖已初始化的 I2C 总线）
    ret = es8311_codec_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "es8311_codec_init failed: %d", (int)ret);
        return ret;
    }

    // 3. 启用板载扬声器功放（PCA9557 IO1 = PA_EN）
    //    pca9557_init() 将 PA_EN 初始化为低电平（功放关闭），此时 ES8311 输出的
    //    音频信号无法到达扬声器，所以即使 I2S 和 ES8311 都正常也听不到声音。
    //    此处在音频通路就绪后拉高 PA_EN 打开板载功放。
    //    该步骤为"尽力而为"：失败仅告警，不影响后续队列与任务创建。
    esp_err_t pa_ret = pca9557_set_output_state(1, 1);
    if (pa_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "启用扬声器功放(PA_EN)失败: %d（请检查PCA9557/音频硬件）", (int)pa_ret);
    }
    else
    {
        ESP_LOGI(TAG, "扬声器功放(PA_EN)已启用");
    }

    // 4. 创建播报队列（深度 4）
    voice_queue = xQueueCreate(4, sizeof(voice_id_t));
    if (!voice_queue)
    {
        ESP_LOGE(TAG, "voice queue create failed");
        return ESP_FAIL;
    }

    // 5. 创建播报任务（栈 4KB）
    //    绑定到 Core1：与摄像头采集、LCD 显示同核；AI 推理任务独占 Core0。
    //    优先级 5（与 LCD 任务同级）：Core1 上 voice 与 LCD 按时间片轮转，
    //    I2S DMA 缓冲（约 1s）足以吸收 LCD 推屏的短促抢占，不会欠喂。
    //    这样一段音频播放（2~5s）期间不会抢占 Core0 的 AI 推理，
    //    人脸识别节奏不被语音打断，从根本上消除"识别混乱卡顿"。
    if (xTaskCreatePinnedToCore(voice_task, "voice_task", 4 * 1024, NULL, 5, NULL, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "voice task create failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "语音播报模块初始化完成");
    return ESP_OK;
}

/**
 * @brief 异步播报指定语音片段
 *
 * 播报策略：
 *   - 当前正在播报某段（voice_playing==true）：直接丢弃，不入队
 *   - 队列满（投递失败）：丢弃
 *   - 上一段已播完（voice_playing==false）且队列空：投递成功
 * 这样保证一段语音完整播放完才会触发下一段，避免 I2S 欠喂/卡死。
 */
void voice_play(voice_id_t id)
{
    if (!voice_queue || id >= VOICE_ID_MAX)
        return;

    // 播报占用期间拒收：避免半截打断导致 I2S DMA 写队列错乱
    if (voice_playing)
    {
        ESP_LOGD("voice_player", "正在播报中，丢弃语音 %02d 请求", (int)id + 1);
        return;
    }

    // 非阻塞投递：队列满（异常情况下）则直接丢弃本次请求
    xQueueSend(voice_queue, &id, 0);
}
