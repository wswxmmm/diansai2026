/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "mpu_port.h"

// 外部声明 mpu_port.c 里的时间戳变量
extern volatile uint32_t sys_tick_ms;

// SysTick 中断服务函数 (每 1ms 触发一次)
void SysTick_Handler(void) {
    sys_tick_ms++;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    OLED_Clear();
    while(DMP_Init());
    float pitch = 0, roll = 0, yaw = 0;
    
    while (1) {
        char oled_str[50];
        // int int_a = 20;
        // sprintf(oled_str, "Integer: %d", int_a);
        // OLED_ShowString(0, 46, (u8 *)oled_str, 16);
        // OLED_Refresh();
        while(DMP_Read_Data(&pitch, &roll, &yaw));
        sprintf(oled_str, "P: %.2f", pitch);
        OLED_ShowString(0, 0, (u8 *)oled_str, 16);
        sprintf(oled_str, "R: %.2f", roll);
        OLED_ShowString(0, 16, (u8 *)oled_str, 16);
        sprintf(oled_str, "Y: %.2f", yaw);
        OLED_ShowString(0, 32, (u8 *)oled_str, 16);
        OLED_Refresh();
        // delay_ms(100);

        // OLED_ShowString(0, 0, (u8 *)"Hello, TI!", 16);
        // OLED_Refresh();
        // delay_ms(500);
        // DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN);
        // delay_ms(500);
        // DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        // DL_GPIO_setPins(LED_PORT, LED_LED1_PIN);
    }
}
