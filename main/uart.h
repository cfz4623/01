

#ifndef __UART_H__
#define __UART_H__

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void uart_init(uint32_t baud_rate);

#ifdef __cplusplus
}
#endif


#endif
