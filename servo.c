#include "servo.h"

uint16_t current_degree = 90;

void servo_init(void)
{
    uint32_t period; //total cycle 320,000
    uint32_t pulse_width; // when high
    uint32_t match_value; // the drop

    // turn on clock for timer1 and port B
    SYSCTL_RCGCTIMER_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;

    while((SYSCTL_PRTIMER_R & 0x02) == 0){}
    while((SYSCTL_PRGPIO_R & 0x02) == 0){}

    // use PB5 for T1CCP1
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |= 0x00700000;
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AMSEL_R &= ~0x20;

    // disable timer1B while setting up
    TIMER1_CTL_R &= ~0x0100;

    // use 16-bit timer mode
    TIMER1_CFG_R = 4;

    // periodic mode, PWM mode, count down
    TIMER1_TBMR_R = 0x0A;



    // 16 ticks x 20,000 microsecs = 320,000
    period = 320000;

    TIMER1_TBILR_R = period & 0xFFFF;
    TIMER1_TBPR_R = (period >> 16) & 0xFF;

    // start at 90 degrees
    pulse_width = 23200; // 1.5 = 1375 x 16 = 22,000
    match_value = period - pulse_width;

    TIMER1_TBMATCHR_R = match_value & 0xFFFF; //lower 16
    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF; //upper 8

    // enable timer1B
    TIMER1_CTL_R|= 0x0100;
}

void servo_move(uint16_t degrees)
{
    uint32_t period; //total cycle 320,000
    uint32_t pulse_width; //when high
    uint32_t match_value; //signal drop
    //Cybot 08
        //0 deg = .5 milli -> 500 x 16 = 8000
        //90 deg = 1.375 milli -> 1375 x 16 = 22000
        //180 deg = 2.2 milli -> 2200 x 16 = 36000

    //Cybot 15
        //0 deg = 8800
        //90 deg = 22800
        //180 deg = 36800

    //Cybot 27
        //0 deg = 7200
        //90 deg = 21200
        //180 deg = 35200

    //Cybot 21
        //0 deg = 8800
        //90 deg = 23200
        //180 deg = 28800

    if(degrees > 180)
    {
        degrees = 180;
    }

    period = 320000;

    // 0 deg = 16000 ticks, 180 deg = 32000 ticks
    pulse_width = 8800 + ((degrees * 28800) / 180);

    match_value = period - pulse_width;

    TIMER1_TBMATCHR_R = match_value & 0xFFFF;
    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF;

    current_degree = degrees;

    timer_waitMillis(200);
}

