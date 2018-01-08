// Author : Amol A More
// Date : April 14, 2017

#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include "Servo.h"

#define CLOCKWISEPERIOD 15975
#define ACLOCKWISEPERIOD 16275
#define STOPPERIOD 16125
#define CLOCKWISEDUTY 975
#define ACLOCKWISEDUTY 1275
#define STOPDUTY 1125

Timer_A_PWMConfig pwmConfigClockWise1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        CLOCKWISEPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        CLOCKWISEDUTY
};

Timer_A_PWMConfig pwmConfigClockWise2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        CLOCKWISEPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        CLOCKWISEDUTY
};

Timer_A_PWMConfig pwmConfigAClockWise1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        ACLOCKWISEPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        ACLOCKWISEDUTY
};

Timer_A_PWMConfig pwmConfigAClockWise2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        ACLOCKWISEPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        ACLOCKWISEDUTY
};

Timer_A_PWMConfig pwmConfigStop1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        STOPPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        STOPDUTY
};

Timer_A_PWMConfig pwmConfigStop2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        STOPPERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        STOPDUTY
};

void PWMConfig(void)
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);

}
void forward(void)
{

    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigClockWise1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigAClockWise2);
}

void backward(void)
{

    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigAClockWise1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigAClockWise2);
}

void rotateRight(void)
{

    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigClockWise1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigAClockWise2);
}

void rotateLeft(void)
{

    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigAClockWise1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigClockWise2);
}

void stop(void)
{

    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigStop1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigStop2);
}
