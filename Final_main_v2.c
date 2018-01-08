/*
 * hello.c
 *
 *  Created on: Apr 29, 2017
 *      Author 1: Sourindu Chatterjee
 *      Author 2: Amol More
 *      Author 3: Subhendu Mishra
 *      Author 4: Anirudh Ganesh Panthula
 *
 */


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


volatile uint32_t count; // count is the count for the pulse width from
                         // ultrasonic echo
volatile uint32_t flagf;  // Deciding if the echo is falling or rising edge
volatile uint32_t key;    // Gate Key
volatile double distance;  //Calculates the distance of the obstacle
/*
 * defining flags for deciding the direction of the Bot
 *  volatile uint16_t rightflag;
    volatile uint16_t leftflag;
    volatile uint16_t forwardflag;
 */

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
    /*
     * Initialize the flags
    rightflag=0;
    leftflag=0;
    forwardflag=0;
     */
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

    /*
     * This is the code developed for changing the state of the bot according
     * to the present direction of the Bot.Bot always turns right when it
     * encounters an obstacle and goes around the obstacle
     */
    /*
     *
     * if(distance > 50){              //if distance greater than 50 cm move forward
        forwardflag = 1;                // Bot moving forward
        Semaphore_post(semaphore1);     // Move forward state 00
    }
    else if((distance > 20)&&(distance <= 50)){ // decide to turn left/right
        if((rightflag == 0)&&(leftflag == 0)){ // state 00
            forwardflag = 0;
            rightflag = 1;
            Semaphore_post(semaphore3); // move right
        }
        else if((rightflag == 1)&&(leftflag == 0)){// decide to turn left/right
             forwardflag = 0;
             leftflag = 1;
             Semaphore_post(semaphore4);// state 10 turn left
        }
        else if((rightflag == 1)&&(leftflag == 1)){// decide to turn left/right
               forwardflag = 0;
               leftflag = 0;
               Semaphore_post(semaphore4); state 11 turn left
              }
        else {
               forwardflag = 1;
               rightflag = 0;
               leftflag = 0;
               Semaphore_post(semaphore3);state 01 turn right
              }
    }

    else if((distance > 10)&&(distance <= 20)){
        Semaphore_post(semaphore2); // move back
    }
    else{
        Semaphore_post(semaphore0); //stop
    }
     */


}

/*
 * task_Stop uses the first semaphore with the same priority as the other tasks
 *
 */
void task_stop(void){

    while(1){
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        stop();
    }
}

/*
 * task_forward uses the binary semaphore1 with the same priority as the
 * other tasks
 */
void task_forward(void){

    while(1){
        Semaphore_pend(semaphore1, BIOS_WAIT_FOREVER);
        forward();
    }
}

/*
 * task_backward moves the bot in backward direction using the semaphore2 with
 * the same priority as the other tasks
 */
void task_backward(void){

    while(1){
        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);
        backward();
    }
}

/*
 * task_rotateRight uses semaphore3 to turn the Bot right direction with
 * reversing the direction of right wheel. It has the same priority as the
 * tasks
 */
void task_rotateRight(void){

    while(1){
        Semaphore_pend(semaphore3, BIOS_WAIT_FOREVER);
        rotateRight();
    }
}

/*
 * task_rotateLeft uses semaphore4 to rotate the Bot left by turning the left
 * wheel in reverse direction and the right wheel in forward direction
 */
void task_rotateLeft(void){

    while(1){
        Semaphore_pend(semaphore4, BIOS_WAIT_FOREVER);
        rotateLeft();
    }
}

/*
 * Uses the echo signal from the pin 1.6 to get hardware interrupt with rising edge
 *
 */
void ultrasonic_Hwi(void) //Port configured Hwi interrupt number 51
{
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6); // Interrupt with rising edge P1.6
    if(flagf == 0 ){
        MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE); // start
        flagf = 1;                                                     // counter for pulse width
         // change the interrupt for the falling edge
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);

    }
    else{
        flagf = 0;
        MAP_Timer_A_stopTimer(TIMER_A1_BASE);   // stop timer to get the pulse width
        count = MAP_Timer_A_getCounterValue(TIMER_A1_BASE); //get count value
        MAP_Timer_A_clearTimer(TIMER_A1_BASE);  //clear timer for next echo
        // Interrupt change for the rising edge
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    }

    Swi_post(swi0); // post the swi0 for deciding which task should run
}

void config_IO(void){ // config functions
    PWMConfig();
    UltraConfig();
}
