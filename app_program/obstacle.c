#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "app_config.h"
#include "obstacle.h"

/* RP2040 PWM slice registers. */
#define PWM_SLICE_FROM_GPIO(pin)    (((pin) >> 1) & 0x07U)

#define PWM_DIV_REG(pin) \
    (PWM_BASE + PWM_CHx_DIV + \
    (PWM_SLICE_FROM_GPIO(pin) * 0x14U))

#define PWM_CTR_REG(pin) \
    (PWM_BASE + PWM_CHx_CTR + \
    (PWM_SLICE_FROM_GPIO(pin) * 0x14U))


/* ================================================================
 * MICROSECOND TIMER
 * ================================================================
 */

static UW time_us_32_local(void)
{
    return in_w(TIMER_TIMERAWL);
}


/* ================================================================
 * PWM HARDWARE INITIALISATION
 * ================================================================
 */

static void pwm_hw_ensure_ready(void)
{
    /*
     * Only release the PWM peripheral from reset when necessary.
     *
     * IMPORTANT:
     *
     * Do NOT reset the whole PWM peripheral here.
     *
     * The motor-control module may already be using other PWM slices.
     * Resetting PWM here would also reset the motor PWM configuration.
     */
    if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U)
    {
        clr_w(
            RESETS_RESET,
            RESETS_RESET_PWM
        );

        while (
            (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U
        )
        {
        }
    }
}


/* ================================================================
 * OBSTACLE HARDWARE INITIALISATION
 * ================================================================
 */

ER obstacle_init(void)
{
    ER err;

    /* ------------------------------------------------------------
     * Ultrasonic trigger pin
     * ------------------------------------------------------------
     */

    err = gpio_set_pin(
        PIN_ULTRASONIC_TRIG,
        GPIO_MODE_OUT
    );

    if (err < E_OK)
    {
        return err;
    }

    (void)gpio_set_val(
        PIN_ULTRASONIC_TRIG,
        0
    );


    /* ------------------------------------------------------------
     * Ultrasonic echo pin
     * ------------------------------------------------------------
     */

    err = gpio_set_pin(
        PIN_ULTRASONIC_ECHO,
        GPIO_MODE_IN
    );

    if (err < E_OK)
    {
        return err;
    }


    /* ------------------------------------------------------------
     * Servo PWM
     * ------------------------------------------------------------
     */

    pwm_hw_ensure_ready();


    err = pwm_set_pin(
        PIN_SERVO
    );

    if (err < E_OK)
    {
        return err;
    }


    /*
     * Stop this PWM slice while configuring it.
     */
    (void)pwm_set_enabled(
        PIN_SERVO,
        FALSE
    );


    /*
     * clk_sys = 125 MHz.
     *
     * Divider = 125
     *
     * 125 MHz / 125 = 1 MHz
     *
     * Therefore:
     *
     * 1 PWM count ≈ 1 microsecond
     */
    out_w(
        PWM_DIV_REG(PIN_SERVO),
        (125U << 4)
    );


    /*
     * Start counter from zero.
     */
    out_w(
        PWM_CTR_REG(PIN_SERVO),
        0U
    );


    /*
     * Standard servo period:
     *
     * 20,000 us = 20 ms
     *            = 50 Hz
     *
     * Counter starts at zero, therefore wrap = 19999.
     */
    (void)pwm_set_wrap(
        PIN_SERVO,
        19999U
    );


    /*
     * Initial position:
     *
     * 1500 us ≈ 90 degrees / centre.
     */
    (void)pwm_set_cc(
        PIN_SERVO,
        1500U
    );


    /*
     * Start servo PWM.
     */
    (void)pwm_set_enabled(
        PIN_SERVO,
        TRUE
    );


    return E_OK;
}


/* ================================================================
 * SERVO CONTROL
 * ================================================================
 */

ER servo_set_angle(int angle)
{
    UW pulse_width;


    /*
     * Protect against invalid angles.
     */
    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > 180)
    {
        angle = 180;
    }


    /*
     * Approximate standard servo mapping:
     *
     *   0 deg   -> 1000 us
     *  90 deg   -> 1500 us
     * 180 deg   -> 2000 us
     */
    pulse_width =
        1000U +
        (UW)((angle * 1000) / 180);


    /*
     * Return PWM result so the caller can detect an error.
     */
    return pwm_set_cc(
        PIN_SERVO,
        pulse_width
    );
}


/* ================================================================
 * ULTRASONIC DISTANCE READING
 * ================================================================
 */

float ultrasonic_read_cm(void)
{
    UW start_time;
    UW timeout_start;
    UW pulse_width;


    /* ------------------------------------------------------------
     * Generate ultrasonic trigger pulse
     * ------------------------------------------------------------
     */

    (void)gpio_set_val(
        PIN_ULTRASONIC_TRIG,
        0
    );

    WaitUsec(2);


    (void)gpio_set_val(
        PIN_ULTRASONIC_TRIG,
        1
    );

    WaitUsec(10);


    (void)gpio_set_val(
        PIN_ULTRASONIC_TRIG,
        0
    );


    /* ------------------------------------------------------------
     * Wait for ECHO rising edge
     * ------------------------------------------------------------
     *
     * Timeout after 30 ms so this RTOS task cannot wait forever
     * when the sensor is disconnected or no echo is received.
     */

    timeout_start = time_us_32_local();


    while (
        gpio_get_val(PIN_ULTRASONIC_ECHO) == 0U
    )
    {
        if (
            (UW)(
                time_us_32_local() -
                timeout_start
            ) > 30000U
        )
        {
            return -1.0f;
        }
    }


    /*
     * ECHO has gone HIGH.
     *
     * Start timing the pulse width.
     */
    start_time = time_us_32_local();


    /* ------------------------------------------------------------
     * Wait for ECHO falling edge
     * ------------------------------------------------------------
     */

    timeout_start = start_time;


    while (
        gpio_get_val(PIN_ULTRASONIC_ECHO) != 0U
    )
    {
        if (
            (UW)(
                time_us_32_local() -
                timeout_start
            ) > 30000U
        )
        {
            return -1.0f;
        }
    }


    /*
     * Calculate ECHO pulse duration.
     */
    pulse_width =
        (UW)(
            time_us_32_local() -
            start_time
        );


    /*
     * HC-SR04-style conversion:
     *
     * distance in cm ≈ pulse duration / 58
     */
    return ((float)pulse_width) / 58.0f;
}