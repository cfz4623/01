/**
 * @file es8311_codec.h
 * @brief ES8311 音频编解码芯片自实现驱动头文件
 *
 * 功能说明：
 *   1. 根据 i2c.h 中的 I2C_VERSION 宏，在编译期选择两套实现：
 *      - I2C_VERSION_1（旧版驱动）：复用 espressif/es8311 组件初始化；
 *      - I2C_VERSION_2（新版驱动）：该组件仅支持旧版 I2C，故改为
 *        直接通过寄存器操作完成初始化；
 *   2. 默认配置：16kHz 采样率、6.144MHz 主时钟（384 倍频）、
 *      16bit 数据宽度、I2S 从机模式、音量 70%。
 *
 * 使用说明：
 *   - 必须先调用 i2c_init() 成功初始化 I2C 总线后，
 *     再调用 es8311_codec_init() 完成芯片初始化。
 *
 * @note ES8311 器件地址由 CE 引脚决定：CE 拉低为 0x18，CE 拉高为 0x19。
 *       本驱动默认使用 0x18。
 */

#ifndef __ES8311_CODEC_H__
#define __ES8311_CODEC_H__

#include "i2c.h"   /* 依赖自定义 I2C 头文件，提供 esp_err_t、I2C 版本宏等定义 */

/* ES8311 器件 I2C 地址：CE 引脚拉低时地址为 0x18 */
#define ES8311_DEVICE_ADDR     (0x18)

/* 音频播放音量：取值范围 0~100，默认 70（对应 DAC 音量寄存器 0x32 的值 0xB2） */
#define ES8311_VOICE_VOLUME    (70)

/* ====================== 函数声明（extern "C" 防止 C++ 名字修饰） ====================== */
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 初始化 ES8311 音频编解码芯片
 * @return esp_err_t 返回 ESP_OK 表示初始化成功，否则返回对应错误码
 *
 * 功能：完成 ES8311 芯片的完整初始化流程：
 *   1. 复位芯片（寄存器 0x00 复位序列）；
 *   2. 配置时钟源与分频系数（16kHz 采样率、6.144MHz MCLK）；
 *   3. 配置音频格式（I2S 从机模式、16bit 数据宽度）；
 *   4. 上电模拟电路、使能 DAC/耳机驱动、配置 ADC 滤波器；
 *   5. 配置麦克风（禁用）与 DAC 播放音量（70%）。
 *
 * 注意：调用前必须先调用 i2c_init() 完成 I2C 总线初始化。
 */
extern esp_err_t es8311_codec_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ES8311_CODEC_H__ */
