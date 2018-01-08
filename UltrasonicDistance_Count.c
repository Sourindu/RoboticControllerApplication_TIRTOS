/*
 * -------------------------------------------
 *    MSP432 DriverLib - v4_00_00_11
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
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
 * --/COPYRIGHT--*/
/********************************************************************************
 * MSP432 Timer32 - Free Run One Shot
 *
 * Description: In this very simple code example, one of the Timer32 modules
 * is setup in 32-bit free-run mode and started in one shot mode. This means
 * that the timer starts at UINT32_MAX (0xFFFFFFFF) and runs to 0 . Once
 * the timer reaches zero, it halts (one shot). The Timer32 is sourced
 * by MCLK and in this example is configured to have a prescaler of 256 every
 * one tick of the Timer32 module is 256 ticks of MCLK).
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *
 * Author: Timothy Logan
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/driverlib/timer32.h>

/* Standard Includes */
#include <stdint.h>
#include "stdio.h"
#include <stdbool.h>
#include "stdlib.h"

void config_IO(void);

volatile uint16_t flag=0;

volatile uint32_t count;

Timer_A_PWMConfig pwmConfig =
{
         TIMER_A_CLOCKSOURCE_SMCLK,
         TIMER_A_CLOCKSOURCE_DIVIDER_3,
         38001,
         TIMER_A_CAPTURECOMPARE_REGISTER_1,
         TIMER_A_OUTPUTMODE_SET_RESET,
         1
};

Timer_A_ContinuousModeConfig contmodeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_2,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
};

int main(void)
{
    /* Holding the Watchdog */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_enableSleepOnIsrExit();

    config_IO();
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableMaster();
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &contmodeConfig);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    printf("Main Done \n");

    MAP_Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
                TIMER32_FREE_RUN_MODE);
    MAP_Timer32_setCount(TIMER32_BASE,UINT32_MAX);
    MAP_Timer32_startTimer(TIMER32_BASE, false);

    while(1){
        MAP_PCM_gotoLPM0();
    }
}

void PORT1_IRQHandler(void){
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);
    if(flag == 0 ){
        MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
        flag = 1;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    }
    else{
        flag = 0;
        MAP_Timer_A_stopTimer(TIMER_A1_BASE);
        count = MAP_Timer_A_getCounterValue(TIMER_A1_BASE);
        MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &contmodeConfig);
        MAP_Timer_A_clearTimer(TIMER_A1_BASE);
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    }
}

void config_IO(void)
{
    printf("Config Done \n");
    MAP_GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
}
