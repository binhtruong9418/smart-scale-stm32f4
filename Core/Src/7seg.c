/*
* 7seg.cpp
*
* Created on: Mar 20, 2025
* Author: TrungNL
* Modified: Added decimal point support for 2-digit display
*/

#include "main.h"
#include "7seg.h"
#include "stm32f4xx_hal.h"

int DisplayValue;
int pos;
int ShowDecimalPoint = 0; // 0: không hiển thị dấu thập phân, 1: hiển thị ở digit đầu tiên

unsigned char Mask[] = {0b00111111, //0 a = lsb, p = msb
                       0b00000110, //1
                       0b01011011, //2
                       0b01001111, //3
                       0b01100110, //4
                       0b01101101, //5
                       0b01111101, //6
                       0b00000111, //7
                       0b01111111, //8
                       0b01101111};//9

// Hàm gốc - hiển thị số nguyên từ 0-99
void Set7SegDisplayValue(int val)
{
    if(val > 99) val = 99;
    if(val < 0) val = 0;

    DisplayValue = val;
    pos = 0;
    ShowDecimalPoint = 0;
}

// Hàm hiển thị số thập phân 1 chữ số (0.0 - 9.9)
void Set7SegDisplayDecimal(float val)
{
    if(val > 9.9) val = 9.9;
    if(val < 0.0) val = 0.0;

    // Chuyển đổi: 1.2 -> 12, 9.9 -> 99
    DisplayValue = (int)(val * 10 + 0.5); // +0.5 để làm tròn
    pos = 0;
    ShowDecimalPoint = 1; // Hiển thị dấu thập phân ở digit đầu tiên
}

// Hàm hiển thị số với dấu thập phân tùy chọn
void Set7SegDisplayWithDecimal(int val, int show_decimal)
{
    if(val > 99) val = 99;
    if(val < 0) val = 0;

    DisplayValue = val;
    pos = 0;
    ShowDecimalPoint = show_decimal;
}

void Run7SegDisplay()
{
    unsigned char val;
    int show_decimal_now = 0;

    pos++;
    HAL_GPIO_WritePin(PORT_7SEG_CONTROL0, PIN_7SEG_CONTROL0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT_7SEG_CONTROL1, PIN_7SEG_CONTROL1, GPIO_PIN_RESET);

    if (pos & 0x1) {
        // Digit đầu tiên (bên phải) - hàng đơn vị
        val = Mask[DisplayValue % 10];
        show_decimal_now = 0; // Không hiển thị dấu thập phân ở digit này
    } else {
        // Digit thứ hai (bên trái) - hàng chục
        val = Mask[(DisplayValue / 10) % 10];
        // Hiển thị dấu thập phân ở digit này nếu cần (sau số hàng chục)
        if(ShowDecimalPoint) {
            show_decimal_now = 1;
        }
    }

    // Thiết lập segment P (decimal point)
    if (show_decimal_now)
        HAL_GPIO_WritePin(PORT_7SEG_P, PIN_7SEG_P, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_P, PIN_7SEG_P, GPIO_PIN_RESET);

    // Thiết lập các segment khác (G, F, E, D, C, B, A)
    if (val & 0x40)
        HAL_GPIO_WritePin(PORT_7SEG_G, PIN_7SEG_G, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_G, PIN_7SEG_G, GPIO_PIN_RESET);

    if (val & 0x20)
        HAL_GPIO_WritePin(PORT_7SEG_F, PIN_7SEG_F, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_F, PIN_7SEG_F, GPIO_PIN_RESET);

    if (val & 0x10)
        HAL_GPIO_WritePin(PORT_7SEG_E, PIN_7SEG_E, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_E, PIN_7SEG_E, GPIO_PIN_RESET);

    if (val & 0x8)
        HAL_GPIO_WritePin(PORT_7SEG_D, PIN_7SEG_D, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_D, PIN_7SEG_D, GPIO_PIN_RESET);

    if (val & 0x4)
        HAL_GPIO_WritePin(PORT_7SEG_C, PIN_7SEG_C, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_C, PIN_7SEG_C, GPIO_PIN_RESET);

    if (val & 0x2)
        HAL_GPIO_WritePin(PORT_7SEG_B, PIN_7SEG_B, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_B, PIN_7SEG_B, GPIO_PIN_RESET);

    if (val & 0x1)
        HAL_GPIO_WritePin(PORT_7SEG_A, PIN_7SEG_A, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_A, PIN_7SEG_A, GPIO_PIN_RESET);

    // Kích hoạt digit tương ứng
    if (pos & 0x1)
        HAL_GPIO_WritePin(PORT_7SEG_CONTROL0, PIN_7SEG_CONTROL0, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PORT_7SEG_CONTROL1, PIN_7SEG_CONTROL1, GPIO_PIN_SET);
}
