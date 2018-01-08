/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/*
 *  ======== main ========
 *  RTOS-Usense
 *  Sourindu Chatterjee
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>
/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
/* Example/Board Header files */
#include "Board.h"

volatile uint16_t flagf=0;

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


Int main()
{
    /* Call driver init functions */
    Board_initGeneral();
    UltraConfig();

    System_printf("Blink the LED\n");
    System_flush();

    BIOS_start();
    return(0);
}

void myHwiIsr(void) //Hwi
{
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);
    if(flagf == 0 ){
        MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
        flagf = 1;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    }
    else{
        flagf = 0;
        MAP_Timer_A_stopTimer(TIMER_A1_BASE);
        count = MAP_Timer_A_getCounterValue(TIMER_A1_BASE);
        MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &contmodeConfig);
        MAP_Timer_A_clearTimer(TIMER_A1_BASE);
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    }
}
void mySwiIsr(void) //Swi
{

}
void myTaskLed1(void) //Task1
{

}
void Nobe(void) //Idle Function
{

    while(1) System_flush();
}

UltraConfig(void)
{
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableMaster();
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &contmodeConfig);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    MAP_GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
}
