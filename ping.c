/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"

// Global shared variables
volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile enum {LOW, HIGH, DONE} g_state = LOW; // State of ping echo pulse
int over = 0;
static float info[4];

void ping_init(void) {
    // Enable GPIOB and Timer3 clocks
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCTIMER_R |= 0x08;

    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRTIMER_R & 0x08) == 0) {}


    // Configure PB3 as input with timer alternate function
    GPIO_PORTB_DIR_R &= ~0x08;       // input
    GPIO_PORTB_AFSEL_R |= 0x08;      // enable AF
    GPIO_PORTB_PCTL_R &= ~0x0000F000;
    GPIO_PORTB_PCTL_R |= 0x00007000; // T3CCP1
    GPIO_PORTB_DEN_R |= 0x08;        // digital enable

    // Configure Timer3B in capture mode, edge-time
    TIMER3_CTL_R &= ~0x100;          // disable timer
    TIMER3_CFG_R = 0x4;              // 16-bit timer with prescaler
    TIMER3_TBMR_R = 0x07;            // capture mode, edge-time

   // 24-bit timer max
    TIMER3_TBILR_R = 0xFFFF;
    TIMER3_TBPR_R  = 0xFF;

    TIMER3_CTL_R |= 0xC00;            // both edges


    TIMER3_ICR_R = 0x400;            // clear capture interrupt
    TIMER3_IMR_R |= 0x400;           // enable capture interrupt


    NVIC_EN1_R |= 0x10;
    //    NVIC_PRI9_R &= ~0x000000E0;
    //    NVIC_PRI9_R |= 0x00000020;

    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntMasterEnable();

    TIMER3_CTL_R |= 0x100;           // enable timer
}

void ping_trigger(void) {
    g_state = LOW;

    // Disable timer and interrupt temporarily
    TIMER3_CTL_R &= ~0x100;
    TIMER3_IMR_R &= ~0x400;

    // Prepare PB3 as output for trigger
    GPIO_PORTB_AFSEL_R &= ~0x08;
    GPIO_PORTB_DIR_R |= 0x08;

    // Send trigger pulse (min 10 µs)
    GPIO_PORTB_DATA_R &= ~0x08;
    timer_waitMicros(2);

    GPIO_PORTB_DATA_R |= 0x08;
    timer_waitMicros(12);
    GPIO_PORTB_DATA_R &= ~0x08;

    // Restore PB3 to input/AF for timer capture
//    GPIO_PORTB_PCTL_R &= ~0x0000F000;
//    GPIO_PORTB_PCTL_R |= 0x00007000;

    GPIO_PORTB_DIR_R &= ~0x08;

    // Clear any pending interrupt
    TIMER3_ICR_R |= 0x400;

    GPIO_PORTB_AFSEL_R |= 0x08;
    // Re-enable timer interrupt and timer
    TIMER3_IMR_R |= 0x400;
    TIMER3_CTL_R |= 0x100;
}

void TIMER3B_Handler(void) {
    if (TIMER3_MIS_R & 0x400) {
        TIMER3_ICR_R |= 0x400; // clear interrupt

        if (g_state == LOW) {
            g_start_time = TIMER3_TBR_R;
            g_state = HIGH;
        } else if (g_state == HIGH) {
            g_end_time = TIMER3_TBR_R;
            g_state = DONE;
        }
    }
}

float* ping_getDistance(void) {
//    ping_trigger();
    int timeout = 0;
    while (g_state != DONE && timeout < 200000) {
          timeout++;
      }

      if (g_state != DONE) {
          g_state = LOW;
          info[0] = 0;
          info[1] = 0;
          info[2] = 110;
          info[3] = over;
          return info;
      }


    uint32_t pulse;
    if (g_start_time >= g_end_time) {
           pulse = g_start_time - g_end_time;
           over++;
       } else {
           pulse = (0x1000000) + g_start_time - g_end_time;
       }
    float time_ms = ((float)pulse/16000000.0f) * 1000.0f;         // ticks to millisecs
    float distance_cm = ((pulse / 16000000.0) * 34300.0 / 2.0); // sound travel one way
    info[0] = pulse;
    info[1] = time_ms;
    info[2] = distance_cm;
    info[3] = over;
    return info;
}
