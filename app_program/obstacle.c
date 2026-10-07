#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "obstacle.h"

#define ULTRASONIC_ECHO_PIN     16
#define ULTRASONIC_TRIG_PIN     17
#define SERVO_PIN               12

/* RP2040 PWM slice registers. GP12 belongs to PWM slice 6, channel A. */
#define PWM_SLICE_FROM_GPIO(pin)    (((pin) >> 1) & 0x07U)
#define PWM_DIV_REG(pin)            (PWM_BASE + PWM_CHx_DIV + (PWM_SLICE_FROM_GPIO(pin) * 0x14U))
#define PWM_CTR_REG(pin)            (PWM_BASE + PWM_CHx_CTR + (PWM_SLICE_FROM_GPIO(pin) * 0x14U))

static UW time_us_32_local(void)
{
    return in_w(TIMER_TIMERAWL);
}

ER obstacle_init(void)
{
    ER err;

    /* Ultrasonic trigger -> output. */
    err = gpio_set_pin(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUT);
    if (err < E_OK)
    {
        return err;
    }
    gpio_set_val(ULTRASONIC_TRIG_PIN, 0);

    /* Ultrasonic echo -> input. */
    err = gpio_set_pin(ULTRASONIC_ECHO_PIN, GPIO_MODE_IN);
    if (err < E_OK)
    {
        return err;
    }

    /*
     * This project's USE_PTMR is 0, so startup does not release the PWM
     * peripheral from reset. Release it here before using the BSP PWM calls.
     */
    set_w(RESETS_RESET, RESETS_RESET_PWM);
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U)
    {
    }

    /* GP12 -> PWM. */
    err = pwm_set_pin(SERVO_PIN);
    if (err < E_OK)
    {
        return err;
    }

    pwm_set_enabled(SERVO_PIN, FALSE);

    /*
     * clk_sys = 125 MHz. RP2040 PWM divider uses 8.4 fixed-point format.
     * Divider 125.0 -> 1 MHz PWM counter -> 1 count = 1 us.
     */
    out_w(PWM_DIV_REG(SERVO_PIN), (125U << 4));
    out_w(PWM_CTR_REG(SERVO_PIN), 0U);

    /* 20,000 us period -> 50 Hz servo signal. */
    pwm_set_wrap(SERVO_PIN, 19999U);

    /* Start near centre: 1500 us pulse. */
    pwm_set_cc(SERVO_PIN, 1500U);
    pwm_set_enabled(SERVO_PIN, TRUE);

    return E_OK;
}

void servo_set_angle(int angle)
{
    UW pulse_width;

    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > 180)
    {
        angle = 180;
    }

    /* 0 deg ~= 1000 us, 90 deg ~= 1500 us, 180 deg ~= 2000 us. */
    pulse_width = 1000U + (UW)((angle * 1000) / 180);

    pwm_set_cc(SERVO_PIN, pulse_width);
}

float ultrasonic_read_cm(void)
{
    UW start_time;
    UW timeout_start;
    UW pulse_width;

    /* Send a 10 us trigger pulse. */
    gpio_set_val(ULTRASONIC_TRIG_PIN, 0);
    WaitUsec(2);

    gpio_set_val(ULTRASONIC_TRIG_PIN, 1);
    WaitUsec(10);

    gpio_set_val(ULTRASONIC_TRIG_PIN, 0);

    /* Wait for ECHO to rise, but do not block forever. */
    timeout_start = time_us_32_local();
    while (gpio_get_val(ULTRASONIC_ECHO_PIN) == 0U)
    {
        if ((UW)(time_us_32_local() - timeout_start) > 30000U)
        {
            return -1.0f;
        }
    }

    start_time = time_us_32_local();

    /* Wait for ECHO to fall. */
    timeout_start = start_time;
    while (gpio_get_val(ULTRASONIC_ECHO_PIN) != 0U)
    {
        if ((UW)(time_us_32_local() - timeout_start) > 30000U)
        {
            return -1.0f;
        }
    }

    pulse_width = (UW)(time_us_32_local() - start_time);

    /* HC-SR04-style conversion: round-trip echo time / 58 ~= cm. */
    return ((float)pulse_width) / 58.0f;
}
