#ifndef __LED_H__
#define __LED_H__

#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_10        // 宏定义：LED 所连接的 GPIO 引脚号（GPIO10）

#ifdef __cplusplus
extern "C" {
#endif

extern void led_init(void);

#ifdef __cplusplus
}
#endif

#endif
