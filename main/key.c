

/**
 * ============================================================
 *  项目名称 : demo1_key
 *  模块名称 : KEY 按键模块 —— 源文件
 *  功能描述 : 实现按键 GPIO 的普通输入初始化、按键电平读取函数，
 *             以及按键下降沿中断模式的初始化与中断服务函数注册
 *  【使用说明】
 *    - key_init()     用于轮询方式读取按键前的初始化；
 *    - key_irq_init() 用于中断方式检测按键，需配合 key_irq_handler() 使用；
 *    - key_read()     用于直接读取按键当前电平状态。
 * ============================================================
 */

#include <stdio.h>                 // 标准输入输出头文件，提供 printf 等函数
#include "freertos/FreeRTOS.h"     // FreeRTOS 内核头文件，提供任务、延时等接口
#include "freertos/task.h"         // FreeRTOS 任务头文件，提供 vTaskDelay 等任务函数
#include "driver/gpio.h"           // GPIO 驱动头文件，提供 gpio 的函数接口
#include "key.h"                   // 自定义 KEY 头文件，提供 key_init 函数声明


/**
 * @brief KEY 初始化函数：将 GPIO 配置为输入模式
 */
void key_init(void)
{
    // 定义 GPIO 配置结构体，设置各配置项
    gpio_config_t gpio = {
        .pin_bit_mask = (1 << KEY1_PIN) | (1 << KEY2_PIN),             // 位掩码：选中 GPIO0 引脚（bit0 置 1）和 GPIO11 引脚（bit11 置 1）
        .mode = GPIO_MODE_INPUT,                 // 模式：输入模式
        .pull_up_en = GPIO_PULLUP_DISABLE,        // 上拉：禁用内部上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // 下拉：禁用内部下拉电阻
        .intr_type = GPIO_INTR_DISABLE,           // 中断：不使用中断（禁用 GPIO 中断）
    };

    // gpio的配置：将上述配置写入硬件寄存器，完成 GPIO 初始化
    gpio_config(&gpio);
}

/**
 * @brief KEY 中断模式下 初始化函数：将 GPIO 配置为中断模式，触发模式为下降沿模式
 */
void IRAM_ATTR key_irq_init(void)
{
    // 定义 GPIO 配置结构体，设置各配置项
    gpio_config_t gpio = {
        .pin_bit_mask = (1 << KEY1_PIN) | (1 << KEY2_PIN),             // 位掩码：选中 GPIO0 引脚（bit0 置 1）和 GPIO11 引脚（bit11 置 1）
        .mode = GPIO_MODE_INPUT,                 // 模式：输入模式
        .pull_up_en = GPIO_PULLUP_DISABLE,        // 上拉：禁用内部上拉电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // 下拉：禁用内部下拉电阻
        .intr_type = GPIO_INTR_NEGEDGE,           // 中断：下降沿触发
    };

    // gpio的配置：将上述配置写入硬件寄存器，完成 GPIO 初始化
    gpio_config(&gpio);

    // 安装中断服务：将中断服务函数添加到中断服务表中
    gpio_install_isr_service(0);

    // 添加中断服务函数：将中断服务函数添加到中断服务表中
    gpio_isr_handler_add(KEY1_PIN, key_irq_handler, (void *)KEY1_PIN);
    gpio_isr_handler_add(KEY2_PIN, key_irq_handler, (void *)KEY2_PIN);
}

/**
 * @brief KEY 读取函数：读取 KEY 状态
 * @return uint8_t KEY 状态:
 * 0 表示 KEY2 未按下，KEY1 未按下；
 * 1 表示 KEY2 未按下，KEY1 按下；
 * 2 表示 KEY2 按下，KEY1 未按下；
 * 3 表示 KEY2 按下，KEY1 按下
 */
uint8_t key_read(void)
{
    uint8_t key_state = 0;

    key_state  = (gpio_get_level(KEY1_PIN)==0)?1<<0:0;
    key_state |= (gpio_get_level(KEY2_PIN)==0)?1<<1:0;
 
    return key_state;
}
