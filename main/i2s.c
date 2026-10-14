/**
 * @file i2s.c
 * @brief I2S 音频外设驱动源文件
 *
 * 功能说明：
 *   1. 本文件实现了 I2S 标准模式发送通道的初始化（主模式、16bit、立体声），
 *      配置 MCLK/BCLK/WS/DOUT 引脚，并与 ES8311 编解码芯片的采样率对齐；
 *   2. 实现音乐播放任务 i2s_music，循环读取内嵌的 canon.pcm 音频数据，
 *      通过 I2S 通道写入 ES8311 驱动扬声器播放；
 *   3. 对外提供 i2s_driver_init() 与 i2s_music_task_start() 两个接口。
 *
 * 使用说明：
 *   - 初始化顺序：先调用 i2s_driver_init()，再调用 es8311_codec_init()
 *     （两者无 I2C 依赖关系，但 ES8311 芯片初始化依赖 I2C，需确保 i2c_init()
 *     已先执行），最后调用 i2s_music_task_start() 启动播放任务。
 *
 * @note 音乐数据 canon.pcm 通过 CMakeLists.txt 中的 EMBED_FILES 指令链接进
 *       固件，编译后生成 _binary_canon_pcm_start / _binary_canon_pcm_end
 *       两个符号，分别指向数据的起始与结束地址。
 */

#include "i2s.h"                 /* 自定义 I2S 头文件，提供引脚宏、采样率宏与函数声明 */
#include "freertos/FreeRTOS.h"   /* FreeRTOS 内核头文件，提供任务、延时等接口 */
#include "freertos/task.h"       /* FreeRTOS 任务头文件，提供 xTaskCreate、vTaskDelay 等 */
#include "esp_log.h"             /* ESP 日志头文件，提供 ESP_LOGE、ESP_LOGI 等宏 */
#include "esp_check.h"           /* ESP 检查头文件，提供 ESP_ERROR_CHECK 等宏 */
#include <stdio.h>               /* 标准输入输出头文件，提供 printf 等函数 */

/* 日志标签：用于 ESP_LOGx 系列日志输出时标识来源 */
static const char *TAG = "i2s_es8311";

/* 错误原因描述数组：下标 0 表示参数非法，下标 1 表示操作超时，
 * 配合 i2s_channel_write 返回值判断具体失败原因 */
static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"};

/* I2S 发送通道句柄：全局静态变量，初始化后供播放任务使用 */
static i2s_chan_handle_t tx_handle = NULL;

/* 引用内嵌的 PCM 音乐数据起始/结束地址：
 * 使用 asm() 将 C 符号映射到链接器生成的二进制符号 */
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[] asm("_binary_canon_pcm_end");

/**
 * @brief 初始化 I2S 音频外设（标准模式发送通道）
 * @return esp_err_t 返回 ESP_OK 表示初始化成功，否则触发断言终止程序
 *
 * 功能：
 *   1. 调用 i2s_new_channel 创建 I2S 发送通道（仅发送、不接收），
 *      并设置 auto_clear 自动清除 DMA 缓冲区中的旧数据；
 *   2. 配置标准模式（Philips I2S 格式）：采样率 16kHz（与 ES8311 一致）、
 *      16bit 数据宽度、立体声双声道，映射 MCLK/BCLK/WS/DOUT 引脚；
 *   3. 设置 MCLK 倍频系数为 384（16kHz * 384 = 6.144MHz，与 ES8311 主时钟一致）；
 *   4. 调用 i2s_channel_init_std_mode 应用配置并使能发送通道。
 *
 * 注意：I2S 默认采样率为 48kHz，而 ES8311 默认采样率为 16kHz，
 *       必须将采样率调整为 16kHz 二者才能正常同步。
 */
esp_err_t i2s_driver_init(void)
{
    /* 配置 I2S 发送通道：指定 I2S 外设编号与主模式角色 */
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    /* 使能自动清除功能：自动清除 DMA 缓冲区中的旧数据 */
    chan_cfg.auto_clear = true;

    /* 创建新的 I2S 通道：只创建发送通道，接收通道传 NULL 表示不使用 */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    /* 配置 I2S 标准模式参数 */
    i2s_std_config_t std_cfg = {
        /* 时钟配置：采样率 16kHz（与 ES8311 保持一致） */
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        /* 时隙配置：16bit 数据宽度、立体声模式
         * 说明：内嵌提示音 PCM 为立体声（150KB÷16000÷4≈2.34s，符合提示音时长），
         * 必须按立体声配置。若误配成 MONO，立体声 PCM 的 L/R 交错数据会被当成
         * 单声道样本序列送入 ES8311，导致音调/音色异常（变音）；
         * 若误配成 MONO 且 PCM 实为单声道，则左右声道错位、音量减半。
         * 故保持 STEREO 与立体声 PCM 严格对应，ES8311 为 I2S 从机自动跟随 WS。 */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        /* GPIO 引脚配置 */
        .gpio_cfg =
        {
            /* 主时钟 MCLK：用于同步数据传输，提供高精度时钟源 */
            .mclk = I2S_MCK_IO,
            /* 位时钟 BCLK：用于同步每个数据位的传输 */
            .bclk = I2S_BCK_IO,
            /* 字选择 WS：用于选择左右声道（字选择时钟） */
            .ws = I2S_WS_IO,
            /* 数据输出 DOUT：向 ES8311 发送音频数据 */
            .dout = I2S_DO_IO,
            /* 数据输入 DIN：-1 表示本示例不使用录音功能 */
            .din = I2S_DI_IO,
            /* 引脚反相标志：全部不反相，保持默认逻辑电平 */
            .invert_flags =
            {
                .mclk_inv = false, /* 主时钟不反相 */
                .bclk_inv = false, /* 位时钟不反相 */
                .ws_inv = false,   /* 字选择不反相 */
            },
        },
    };

    /* 调整主时钟倍频系数为 384：
     * 实际 MCLK 频率 = 采样率 * 384 = 16000 * 384 = 6.144MHz，
     * 与 ES8311 的主时钟输入频率匹配，保证音频采样率正确 */
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    /* 初始化 I2S 通道为标准模式并应用上述全部配置 */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    /* 使能 I2S 发送通道 */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    return ESP_OK;
}

/**
 * @brief 音乐播放任务函数（FreeRTOS 任务）
 * @param args 任务参数指针（本示例未使用，传 NULL）
 * @return 无返回值，任务运行期间若出错将调用 abort() 终止程序
 *
 * 功能：
 *   1. 先禁用发送通道并预加载首段音频数据到 DMA 缓冲区，
 *      使能通道后可立即传输有效数据，消除播放起始延迟；
 *   2. 进入无限循环，将内嵌的 canon.pcm 音频数据循环写入 I2S 通道，
 *      每次写完一帧后延时 1 秒再播放下一帧，实现循环播放。
 *
 * 注意：i2s_channel_write 使用 portMAX_DELAY 等待，若返回错误则根据
 *       错误码打印对应原因并终止程序（abort）。
 */
static void i2s_music(void *args)
{
    /* 定义错误状态变量，初始化为成功 */
    esp_err_t ret = ESP_OK;
    /* 定义已写入字节数变量，初始化为 0 */
    size_t bytes_write = 0;
    /* 定义数据指针，指向音乐 PCM 数据起始地址 */
    uint8_t *data_ptr = (uint8_t *)music_pcm_start;

    /* 可选步骤：禁用 TX 通道并预加载数据，使能通道后可立即传输有效数据 */
    /* 禁用 I2S 发送通道 */
    ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
    /* 预加载数据到 DMA 缓冲区 */
    ESP_ERROR_CHECK(i2s_channel_preload_data(
        tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write));
    /* 移动数据指针，跳过已预加载的数据 */
    data_ptr += bytes_write;

    /* 使能 I2S 发送通道 */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    /* 进入无限循环，持续播放音乐 */
    while (1)
    {
        /* 将音乐数据写入 I2S 通道，使用最大延迟等待传输完成 */
        ret = i2s_channel_write(tx_handle, data_ptr, music_pcm_end - data_ptr,
                                &bytes_write, portMAX_DELAY);
        /* 检查写入操作是否成功 */
        if (ret != ESP_OK)
        {
            /* 打印错误日志：根据返回值选择"参数非法"或"操作超时"原因 */
            ESP_LOGE(TAG, "[music] i2s write failed, %s",
                     err_reason[ret == ESP_ERR_TIMEOUT]);
            /* 终止程序 */
            abort();
        }
        /* 检查是否有数据写入 */
        if (bytes_write > 0)
        {
            /* 打印日志，显示已写入的字节数 */
            ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.",
                     bytes_write);
        }
        else
        {
            /* 打印错误日志，提示音乐播放失败 */
            ESP_LOGE(TAG, "[music] i2s music play failed.");
            /* 终止程序 */
            abort();
        }
        /* 重置数据指针到音乐起始位置，实现循环播放 */
        data_ptr = (uint8_t *)music_pcm_start;
        /* 延迟 1 秒再继续播放下一帧 */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    /* 删除当前任务（正常情况下不会执行到此处） */
    vTaskDelete(NULL);
}

/**
 * @brief 通过 I2S 通道播放一段 PCM 数据（阻塞直到写完）
 *
 * @param data PCM 原始数据指针（16kHz/16bit/双声道）
 * @param len  数据字节长度
 * @return esp_err_t ESP_OK 表示成功，否则返回对应错误码
 *
 * 说明：供语音播报模块（voice_player）播放内嵌提示音使用，
 *       内部循环写入直至全部数据发送完毕。
 */
esp_err_t i2s_write_pcm(const uint8_t *data, size_t len)
{
    /* 参数与通道有效性检查 */
    if (!tx_handle || !data || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t written_total = 0;
    while (written_total < len)
    {
        size_t bytes_write = 0;
        /* 分块写入 I2S，直到剩余数据全部发送完毕 */
        esp_err_t ret = i2s_channel_write(tx_handle,
                                          data + written_total,
                                          len - written_total,
                                          &bytes_write,
                                          portMAX_DELAY);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[pcm] i2s write failed, %s",
                     err_reason[ret == ESP_ERR_TIMEOUT]);
            return ret;
        }
        if (bytes_write == 0)
        {
            ESP_LOGE(TAG, "[pcm] i2s write no data");
            return ESP_FAIL;
        }
        written_total += bytes_write;
    }

    return ESP_OK;
}

/**
 * @brief 创建并启动音乐播放任务
 * @return esp_err_t 返回 ESP_OK 表示任务创建成功，否则返回 ESP_FAIL
 *
 * 功能：调用 xTaskCreate 创建 i2s_music 任务，
 *       任务名称为"i2s_music"，栈大小 4096 字节，优先级 5。
 *
 * 注意：必须在 i2s_driver_init() 成功初始化之后调用。
 */
esp_err_t i2s_music_task_start(void)
{
    /* 创建音乐播放任务：任务函数 i2s_music、名称、栈大小、参数、优先级 */
    BaseType_t ret = xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
    /* 检查任务是否创建成功 */
    if (ret != pdPASS)
    {
        /* 打印错误日志，提示任务创建失败 */
        ESP_LOGE(TAG, "create i2s_music task failed");
        return ESP_FAIL; /* 返回失败错误码 */
    }
    return ESP_OK; /* 返回成功 */
}
