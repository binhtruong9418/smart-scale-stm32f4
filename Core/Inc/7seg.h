/*
 * 7seg.h - Simple driver for 2x7seg modules
 *
 *  Created on: Mar 20, 2025
 *      Author: TrungNL
 */

#ifndef INC_7SEG_H_
#define INC_7SEG_H_

#include "stm32f4xx_hal.h"

/* Module pin definition */
#define PORT_7SEG_A		GPIOE
#define PORT_7SEG_B		GPIOE
#define PORT_7SEG_C		GPIOE
#define PORT_7SEG_D		GPIOE
#define PORT_7SEG_E		GPIOE
#define PORT_7SEG_F		GPIOE
#define PORT_7SEG_G		GPIOE
#define PORT_7SEG_P		GPIOE

#define PIN_7SEG_A		GPIO_PIN_8
#define PIN_7SEG_B		GPIO_PIN_9
#define PIN_7SEG_C		GPIO_PIN_10
#define PIN_7SEG_D		GPIO_PIN_11
#define PIN_7SEG_E		GPIO_PIN_12
#define PIN_7SEG_F		GPIO_PIN_13
#define PIN_7SEG_G		GPIO_PIN_14
#define PIN_7SEG_P		GPIO_PIN_15

#define PORT_7SEG_CONTROL0		GPIOG
#define PIN_7SEG_CONTROL0		GPIO_PIN_2
#define PORT_7SEG_CONTROL1		GPIOG
#define PIN_7SEG_CONTROL1		GPIO_PIN_3

// Hàm gốc - hiển thị số nguyên (0-99)
void Set7SegDisplayValue(int val);

// Hàm mới - hiển thị số thập phân 1 chữ số (0.0-9.9)
void Set7SegDisplayDecimal(float val);

// Hàm mới - hiển thị số với dấu thập phân tùy chọn
void Set7SegDisplayWithDecimal(int val, int show_decimal);

// Hàm chạy hiển thị (không thay đổi)
void Run7SegDisplay(void);

#endif /* INC_7SEG_H_ */
