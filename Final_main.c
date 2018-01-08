/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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
 *  ======== hello.c ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>

/* Example/Board Header files */
#include "Board.h"
#include "Servo.h"
#include "Ultrasonic.h"


volatile uint32_t count;
volatile uint32_t next;
volatile uint32_t flagf;
volatile uint32_t key;
volatile double distance;


void config_IO(void);

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_initGeneral();

    //System_printf("hello world\n");
    config_IO();
    //stop();
    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    //BIOS_exit(0);  /* terminates program and dumps SysMin output */
    BIOS_start();
    return(0);
}


void swi_MotionControl(void){

    key = Hwi_disable();
    distance = count;//((((count/1000)/2)/1000)*340)*100;
    Hwi_restore(key);

    if (distance < 3000){
        Semaphore_post(semaphore0);
    }
    else{
        Semaphore_post(semaphore1);
    }


}

void task_stop(void){

    while(1){
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        stop();
        //Semaphore_post(semaphore3);
    }
}

void task_forward(void){

    while(1){
        Semaphore_pend(semaphore1, BIOS_WAIT_FOREVER);
        forward();
    }
}

void task_backward(void){

    while(1){
        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);
        backward();
    }
}

void task_rotateRight(void){

    while(1){
        Semaphore_pend(semaphore3, BIOS_WAIT_FOREVER);
        rotateRight();
    }
}

void task_rotateLeft(void){

    while(1){
        Semaphore_pend(semaphore4, BIOS_WAIT_FOREVER);
        rotateLeft();
    }
}

void ultrasonic_Hwi(void) //Hwi
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
        MAP_Timer_A_clearTimer(TIMER_A1_BASE);
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    }

    Swi_post(swi0);
}

void config_IO(void){
    PWMConfig();
    UltraConfig();
}
