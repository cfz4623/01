/**
 * @file    main.c
 * @brief   主程序入口：初始化 I2C、PCA9557、LCD 与 DVP 摄像头（GC0308），
 *          随后启动基于 ESP-DL 的人脸识别应用（检测/注册/识别/清空）。
 *
 * @note    使用说明：
 *          - 初始化顺序：i2c_init -> pca9557_init -> lcd_init -> camera_init
 *            -> app_human_face_recognition（有先后依赖，不可颠倒）
 *          - 摄像头电源（DVP_PWDN）由 PCA9557 扩展 IO 第 2 位控制，
 *            camera_init() 内部会拉低上电
 *          - SCCB 控制线复用 I2C0（SIOD=GPIO1 / SIOC=GPIO2），须先 i2c_init()
 *          - 屏幕方向由 lcd.h 的 LCD_ORIENTATION 宏决定（横屏 480x320 / 竖屏
 *            320x480），摄像头按 QVGA(320x240) 采集，由 AI 模块整帧显示
 *          - 人脸识别操作：
 *              - 长按(>1秒)：注册当前人脸到 Flash
 *              - 单击：识别人脸
 *              - 双击：清空所有人脸数据
 *          - 人脸数据保存在 Flash 分区表 "storage" 分区（见 partitions.csv）
 *          - I2C 必须使用新版驱动（i2c.h 中 I2C_VERSION_2，配套 IDF 5.5）
 */

#include "camera.h"            // 自定义摄像头头文件，提供 camera_init、camera_lcd_start 函数声明
#include "freertos/FreeRTOS.h" // FreeRTOS 内核头文件，提供任务、延时等接口
#include "freertos/task.h"     // FreeRTOS 任务头文件，提供 vTaskDelay 等任务函数
#include "i2c.h"               // 自定义 I2C 头文件，提供 i2c_init 函数声明
#include "lcd.h"               // 自定义 LCD 头文件，提供 lcd_init 函数声明
#include "pca9557.h"           // 自定义 PCA9557 头文件，提供 pca9557_init 函数声明
#include "who_human_face_recognition.hpp" // 人脸识别应用头文件，提供 app_human_face_recognition 函数声明
#include <stdio.h>             // 标准输入输出头文件，提供 printf 等函数

/**
 * @brief 应用主函数：程序入口，初始化硬件并启动人脸识别应用
 */
void app_main(void)
{
    // 初始化 I2C
    esp_err_t ret = i2c_init(); // 调用自定义 i2c_init 初始化 I2C 总线（新版驱动），返回错误码
    if (ret != ESP_OK)          // 判断 I2C 初始化是否成功
    {
        printf("i2c_init failed, ret = %d\n", ret); // 初始化失败时打印错误信息及错误码
        return;                                     // 失败则直接退出，不继续执行后续初始化
    }

    // 初始化 PCA9557 IO 扩展芯片
    ret = pca9557_init(); // 初始化 PCA9557（LCD_CS / DVP_PWDN 等控制 IO），返回错误码
    if (ret != ESP_OK)    // 判断 PCA9557 初始化是否成功
    {
        printf("pca9557_init failed, ret = %d\n", ret); // 初始化失败时打印错误信息及错误码
        return;                                         // 失败则直接退出
    }

    // 初始化 LCD 显示屏
    ret = lcd_init();  // 初始化 ST7796 液晶屏（SPI 接口），返回错误码
    if (ret != ESP_OK) // 判断 LCD 初始化是否成功
    {
        printf("lcd_init failed, ret = %d\n", ret); // 初始化失败时打印错误信息及错误码
        return;                                     // 失败则直接退出
    }

    // 设置 LCD 显示屏背景颜色为黑色（与源工程保持一致）
    lcd_set_color(0);

    // 延时，等待摄像头模块上电稳定
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 延时 1000 毫秒

    // 初始化摄像头（内部打开 DVP_PWDN 电源、配置 esp_camera、检测 GC0308）
    ret = camera_init(); // 调用自定义 camera_init 初始化摄像头，返回错误码
    if (ret != ESP_OK)   // 判断摄像头初始化是否成功
    {
        printf("camera_init failed, ret = %d\n", ret); // 初始化失败时打印错误信息及错误码
        return;                                        // 失败则直接退出
    }

    // 启动人脸识别应用：创建采集、AI 检测识别、LCD 显示、按键检测等任务
    app_human_face_recognition();
}
