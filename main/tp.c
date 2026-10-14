/**
 * @file    tp.c
 * @brief   触摸屏驱动封装：负责 FT6x36 触摸屏的初始化（创建 I2C 面板 IO 与触摸设备实例），
 *          并提供获取触摸设备句柄的接口，供上层读取触摸坐标使用。
 *
 * @note    使用说明：
 *          - 依赖组件：esp_lcd_touch、esp_lcd_touch_ft6x36（managed_components 自动拉取）
 *          - 前置条件：必须先调用 i2c_init() 完成 I2C 总线初始化，再调用 tp_init()
 *          - 屏幕分辨率与方向由 lcd.h 的 LCD_ORIENTATION 宏决定（横屏 480x320 /
 *            竖屏 320x480），触摸坐标范围与坐标变换标志跟随该宏自动适配
 *          - 触摸设备地址 0x38（FT6x36），通过宏 ESP_LCD_TOUCH_IO_I2C_FT6x36_CONFIG()
 * 快速获取默认配置
 *          - 未使用硬件复位/中断引脚（rst_gpio_num=-1, int_gpio_num=-1），采用轮询方式读取
 *          - 调用流程：tp_init() -> tp_get_handle() -> esp_lcd_touch_read_data() /
 * esp_lcd_touch_get_coordinates()
 */

#include "tp.h"                // 自定义触摸屏头文件，提供 tp_init、tp_get_handle 函数声明
#include "driver/gpio.h"       // GPIO 驱动头文件，提供 gpio 的函数接口
#include "freertos/FreeRTOS.h" // FreeRTOS 内核头文件，提供任务、延时等接口
#include "freertos/task.h"     // FreeRTOS 任务头文件，提供 vTaskDelay 等任务函数
#include "i2c.h"               // 自定义 I2C 头文件，提供 i2c_init 函数声明
#include "lcd.h"               // 自定义 LCD 头文件，提供 LCD_WIDTH/LCD_HEIGHT 及方向宏
#include <stdio.h>             // 标准输入输出头文件，提供 printf 等函数

// 全局变量定义，用于存储各种硬件设备的句柄
static esp_lcd_touch_handle_t g_ft6x36_handle =
    NULL; // FT6x36触摸屏设备句柄（静态全局，仅本文件内部访问）

/**
 * @brief Touchscreen 初始化函数：初始化触摸屏设备
 */
esp_err_t tp_init(void) // 触摸屏初始化函数：返回 ESP_OK 表示成功
{
    esp_lcd_panel_io_handle_t io_handle =
        NULL; // 定义 I2C 面板 IO 句柄并初始化为空，用于管理 I2C 通信

    // 初始化触摸屏设备，核心要得到触摸屏的设备地址
    esp_lcd_panel_io_i2c_config_t io_config =
        ESP_LCD_TOUCH_IO_I2C_FT6x36_CONFIG(); // 使用 FT6x36 默认宏配置 I2C 面板 IO（设备地址
                                              // 0x38、8位寄存器寻址等）
    io_config.scl_speed_hz =
        I2C_FREQ_HZ; // 宏未配置该字段（默认 0），新版驱动要求 scl_speed_hz>0，
                     // 必须显式指定 SCL 频率（100kHz），否则报 invalid scl frequency

    // 初始化触摸屏设备
    esp_lcd_touch_config_t tp_cfg = {
        // 定义触摸屏配置结构体，设置触摸屏参数
        .x_max = LCD_WIDTH,       // 触摸屏最大 X 坐标（由 LCD_ORIENTATION 宏决定：横屏 480 / 竖屏 320）
        .y_max = LCD_HEIGHT,      // 触摸屏最大 Y 坐标（由 LCD_ORIENTATION 宏决定：横屏 320 / 竖屏 480）
        .rst_gpio_num = -1, // 不使用复位引脚（-1 表示未连接），依赖上电默认状态
        .int_gpio_num = -1, // 不使用中断引脚（-1 表示未连接），采用轮询方式读取
        .levels =           // 配置复位与中断引脚的有效电平
        {
            .reset = 0,     // 复位引脚低电平有效（未接线，仅作默认配置）
            .interrupt = 0, // 中断引脚低电平有效（未接线，仅作默认配置）
        },
        .flags = // 配置触摸坐标的坐标变换标志位（跟随 LCD_ORIENTATION 宏，与屏幕方向保持一致）
        {
            .swap_xy = LCD_SWAP_XY,   // 横屏时交换 X/Y 坐标轴
            .mirror_x = LCD_MIRROR_X, // 与屏幕水平镜像一致
            .mirror_y = LCD_MIRROR_Y, // 与屏幕垂直镜像一致
        },
    };

    esp_err_t ret = ESP_OK; // 定义错误码变量并初始化为成功，用于保存各步骤返回值

#if (I2C_VERSION == I2C_VERSION_1)
    ret = esp_lcd_new_panel_io_i2c(I2C_NUM, &io_config, &io_handle);
#else
    ret = esp_lcd_new_panel_io_i2c_v2(i2c_get_bus_handle(), &io_config, &io_handle);
#endif
    if (ret != ESP_OK) // 判断面板 IO 创建是否成功
    {
        printf("esp_lcd_new_panel_io_i2c failed, ret = %d\n",
               ret); // 创建失败时打印错误信息及错误码
        return ret;  // 失败则返回错误码，终止初始化
    }

    ret = esp_lcd_touch_new_i2c_ft6x36(
        io_handle,
        &tp_cfg,
        &g_ft6x36_handle); // 基于面板 IO 创建 FT6x36 触摸设备实例，保存句柄到全局变量
    if (ret != ESP_OK)     // 判断触摸设备创建是否成功
    {
        printf("esp_lcd_touch_new_i2c_ft6x36 failed, ret = %d\n",
               ret); // 创建失败时打印错误信息及错误码
        return ret;  // 失败则返回错误码，终止初始化
    }
    return ESP_OK; // 全部初始化成功，返回 ESP_OK
}

esp_lcd_touch_handle_t tp_get_handle(void) // 获取触摸屏设备句柄的接口函数
{
    return g_ft6x36_handle; // 返回全局触摸设备句柄，供上层读取触摸数据使用
}
