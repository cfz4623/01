#ifndef __TP_H__
#define __TP_H__
#include "esp_lcd_touch.h"   
#include "esp_lcd_touch_ft6x36.h" // 自定义 Touchscreen 头文件，提供 tp_init 函数声明

#ifdef __cplusplus
extern "C" {
#endif

extern esp_err_t tp_init(void);
extern esp_lcd_touch_handle_t tp_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif
