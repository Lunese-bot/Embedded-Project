#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "motion_control.h"


/*
 * Robo Pico onboard motor driver
 *
 * M1 = Left motor
 * M2 = Right motor
 */

#define M1A     8
#define M1B     9

#define M2A     10
#define M2B     11


void motion_init(void)
{
    /* Configure motor pins as GPIO outputs */
    gpio_set_pin(M1A, GPIO_MODE_OUT);
    gpio_set_pin(M1B, GPIO_MODE_OUT);

    gpio_set_pin(M2A, GPIO_MODE_OUT);
    gpio_set_pin(M2B, GPIO_MODE_OUT);

    /* Safety: motors initially stopped */
    motor_stop();
}


/* =========================
 * LEFT MOTOR
 * ========================= */

void motor_left_forward(void)
{
    gpio_set_val(M1A, 1);
    gpio_set_val(M1B, 0);
}


void motor_left_reverse(void)
{
    gpio_set_val(M1A, 0);
    gpio_set_val(M1B, 1);
}


/* =========================
 * RIGHT MOTOR
 * ========================= */

void motor_right_forward(void)
{
    gpio_set_val(M2A, 1);
    gpio_set_val(M2B, 0);
}


void motor_right_reverse(void)
{
    gpio_set_val(M2A, 0);
    gpio_set_val(M2B, 1);
}


/* =========================
 * STOP BOTH MOTORS
 * ========================= */

void motor_stop(void)
{
    gpio_set_val(M1A, 0);
    gpio_set_val(M1B, 0);

    gpio_set_val(M2A, 0);
    gpio_set_val(M2B, 0);
}