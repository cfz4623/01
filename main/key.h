/**
 * ============================================================
 *  项目名称 : demo1_key
 *  模块名称 : KEY 按键模块 —— 头文件
 *  功能描述 : 定义按键对应的 GPIO 引脚宏，并声明按键初始化、按键
 *             状态读取以及按键中断初始化等对外接口函数
 *  【引脚定义】
 *    KEY1_PIN = GPIO0  （按键按下时输入低电平）
 *    KEY2_PIN = GPIO11 （按键按下时输入低电平）
 * ============================================================
 */

#ifndef __KEY_H__
#define __KEY_H__

#include "driver/gpio.h"

#define KEY1_PIN GPIO_NUM_0        // 宏定义：KEY1 所连接的 GPIO 引脚号（GPIO0）
#define KEY2_PIN GPIO_NUM_11        // 宏定义：KEY2 所连接的 GPIO 引脚号（GPIO11）


/* ====================== 函数声明（extern "C" 防止 C++ 名字修饰） ====================== */
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief KEY 普通模式初始化：将按键引脚配置为输入模式（供轮询方式读取）
 */
extern void key_init(void);

/**
 * @brief KEY 状态读取：轮询读取两个按键的当前电平并组合为状态值
 * @return uint8_t 按键状态：
 *         - bit0 = 1 表示 KEY1 按下
 *         - bit1 = 1 表示 KEY2 按下
 */
extern uint8_t key_read(void);

/**
 * @brief KEY 中断模式初始化：将按键引脚配置为下降沿中断，
 *        并安装 GPIO 中断服务、注册 key_irq_handler 回调
 */
extern void key_irq_init(void);

/**
 * @brief KEY 中断服务函数：在中断上下文中根据触发引脚置位对应按键位，
 *        并通过消息队列通知上层任务（该函数必须位于 IRAM_ATTR 中）
 * @param arg 中断参数，实际传入触发中断的引脚编号
 */
extern void IRAM_ATTR key_irq_handler(void *arg);

#ifdef __cplusplus
}
#endif

#endif
