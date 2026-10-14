#ifndef __I2S_H__
#define __I2S_H__

/* ============================ 文件功能概述 ============================ */
/* 本头文件是 I2S 音频外设驱动模块的接口定义，包含：                        */
/*   1. I2S 外设引脚号宏（MCLK/BCLK/WS/DOUT）；                            */
/*   2. 音频示例配置宏（采样率、MCLK 倍频、MCLK 频率）；                   */
/*   3. I2S 初始化与音乐播放任务启动函数声明。                             */
/* ======================================================================== */

#include "driver/gpio.h"      /* GPIO 驱动头文件，提供 GPIO_NUM_x 引脚宏定义 */
#include "driver/i2s_std.h"   /* I2S 标准模式驱动头文件，提供 i2s_chan_handle_t 等类型 */
#include "esp_err.h"          /* ESP 错误码类型定义 */

/* ====================== I2S 引脚宏定义 ================================= */
#define I2S_NUM         (0)            /* I2S 外设编号：使用 I2S0 */
#define I2S_MCK_IO      (GPIO_NUM_38)  /* 主时钟 MCLK 引脚：GPIO38，用于同步数据传输 */
#define I2S_BCK_IO      (GPIO_NUM_14)  /* 位时钟 BCLK 引脚：GPIO14，用于同步数据位传输 */
#define I2S_WS_IO       (GPIO_NUM_13)  /* 字选择 WS 引脚：GPIO13，用于选择左右声道 */
#define I2S_DO_IO       (GPIO_NUM_45)  /* 数据输出 DO 引脚：GPIO45，向 ES8311 发送音频数据 */
#define I2S_DI_IO       (-1)           /* 数据输入 DI 引脚：-1 表示本示例不使用录音功能 */

/* ====================== 音频示例配置宏 ================================= */
#define EXAMPLE_SAMPLE_RATE     (16000)                       /* 采样率：16kHz（需与 ES8311 一致） */
#define EXAMPLE_MCLK_MULTIPLE   (384)                         /* MCLK 倍频系数：384 倍 */
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE) /* MCLK 频率：6.144MHz */

/* ====================== 函数声明（extern "C" 防止 C++ 名字修饰） ======================================= */
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 初始化 I2S 音频外设
 * @return esp_err_t 返回 ESP_OK 表示初始化成功，否则返回对应错误码
 *
 * 功能：创建 I2S 标准模式发送通道（主模式、16bit、立体声），
 *       配置 MCLK/BCLK/WS/DOUT 引脚并使能通道，供 ES8311 播放音频。
 */
extern esp_err_t i2s_driver_init(void);

/**
 * @brief 创建并启动音乐播放任务
 * @return esp_err_t 返回 ESP_OK 表示任务创建成功，否则返回 ESP_FAIL
 *
 * 功能：创建 i2s_music 任务（栈 4096、优先级 5），循环播放内嵌的
 *       canon.pcm 音频数据。
 */
extern esp_err_t i2s_music_task_start(void);

/**
 * @brief 通过 I2S 通道播放一段 PCM 数据（阻塞直到写完）
 *
 * @param data PCM 原始数据指针（16kHz/16bit/双声道）
 * @param len  数据字节长度
 * @return esp_err_t ESP_OK 表示成功，否则返回对应错误码
 *
 * @note 必须在 i2s_driver_init() 成功之后调用；
 *       内部会分块写入直至全部数据发送完毕
 */
extern esp_err_t i2s_write_pcm(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
