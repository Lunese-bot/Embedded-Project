<<<<<<< Updated upstream
=======
#include <tk/tkernel.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "app_config.h"
#include "motion_control.h"

/*
 * Encoder counts.
 *
 * volatile because these values are repeatedly updated by
 * the motion task and read by other tasks.
 */
static volatile W left_encoder_count = 0;
static volatile W right_encoder_count = 0;

/*
 * Previous encoder A-channel states.
 *
 * Used to detect rising edges.
 */
static UINT left_a_last = 0;
static UINT right_a_last = 0;

/*
 * Prevent motor commands before hardware initialisation.
 */
static BOOL motion_ready = FALSE;


/* ================================================================
 * PWM HARDWARE
 * ================================================================
 */

static void pwm_hw_ensure_ready(void)
{
    /*
     * PWM may still be held in reset when the application starts.
     *
     * IMPORTANT:
     *
     * Do NOT reset the whole PWM peripheral here.
     *
     * The servo and motors use different PWM slices.
     * Resetting the whole PWM peripheral could destroy the
     * servo configuration or another motor configuration.
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
 * MOTOR SPEED HELPERS
 * ================================================================
 */

static int clamp_speed(int speed)
{
    if (speed > 100)
    {
        return 100;
    }

    if (speed < -100)
    {
        return -100;
    }

    return speed;
}


static UW speed_to_cc(int speed)
{
    UW magnitude;

    /*
     * PWM only needs the magnitude.
     *
     * Direction is handled by deciding which motor input receives PWM.
     */
    if (speed < 0)
    {
        speed = -speed;
    }

    magnitude = (UW)speed;

    /*
     * Convert:
     *
     * 0..100 %
     *
     * into:
     *
     * 0..MOTOR_PWM_WRAP
     */
    return (MOTOR_PWM_WRAP * magnitude) / 100U;
}


/*
 * Drive one H-bridge motor pair.
 *
 * Positive:
 *
 *     A = PWM
 *     B = 0
 *
 * Negative:
 *
 *     A = 0
 *     B = PWM
 *
 * Zero:
 *
 *     A = 0
 *     B = 0
 */
static void set_motor_pair(
    UINT pin_a,
    UINT pin_b,
    int speed
)
{
    UW cc;

    speed = clamp_speed(speed);

    cc = speed_to_cc(speed);


    if (speed > 0)
    {
        (void)pwm_set_cc(
            pin_a,
            cc
        );

        (void)pwm_set_cc(
            pin_b,
            0U
        );
    }
    else if (speed < 0)
    {
        (void)pwm_set_cc(
            pin_a,
            0U
        );

        (void)pwm_set_cc(
            pin_b,
            cc
        );
    }
    else
    {
        (void)pwm_set_cc(
            pin_a,
            0U
        );

        (void)pwm_set_cc(
            pin_b,
            0U
        );
    }
}


/* ================================================================
 * INITIALISATION
 * ================================================================
 */

ER motion_init(void)
{
    ER err;


    /*
     * Make sure the RP2040 PWM peripheral is available.
     */
    pwm_hw_ensure_ready();


    /* ============================================================
     * LEFT MOTOR
     *
     * M1A = GP8
     * M1B = GP9
     * ============================================================
     */

    err = pwm_set_pin(
        PIN_MOTOR_LEFT_A
    );

    if (err < E_OK)
    {
        return err;
    }


    err = pwm_set_pin(
        PIN_MOTOR_LEFT_B
    );

    if (err < E_OK)
    {
        return err;
    }


    /*
     * GP8 and GP9 share the same RP2040 PWM slice.
     */
    (void)pwm_set_wrap(
        PIN_MOTOR_LEFT_A,
        MOTOR_PWM_WRAP
    );


    /*
     * Motor starts with zero duty cycle.
     */
    (void)pwm_set_cc(
        PIN_MOTOR_LEFT_A,
        0U
    );

    (void)pwm_set_cc(
        PIN_MOTOR_LEFT_B,
        0U
    );


    /*
     * Enable the PWM slice.
     */
    (void)pwm_set_enabled(
        PIN_MOTOR_LEFT_A,
        TRUE
    );


    /* ============================================================
     * RIGHT MOTOR
     *
     * M2A = GP10
     * M2B = GP11
     * ============================================================
     */

    err = pwm_set_pin(
        PIN_MOTOR_RIGHT_A
    );

    if (err < E_OK)
    {
        return err;
    }


    err = pwm_set_pin(
        PIN_MOTOR_RIGHT_B
    );

    if (err < E_OK)
    {
        return err;
    }


    /*
     * GP10 and GP11 share another PWM slice.
     */
    (void)pwm_set_wrap(
        PIN_MOTOR_RIGHT_A,
        MOTOR_PWM_WRAP
    );


    (void)pwm_set_cc(
        PIN_MOTOR_RIGHT_A,
        0U
    );

    (void)pwm_set_cc(
        PIN_MOTOR_RIGHT_B,
        0U
    );


    (void)pwm_set_enabled(
        PIN_MOTOR_RIGHT_A,
        TRUE
    );


    /* ============================================================
     * LEFT ENCODER
     *
     * GP4 = Encoder A
     * GP5 = Encoder B
     * ============================================================
     */

    err = gpio_set_pin(
        PIN_ENCODER_LEFT_A,
        GPIO_MODE_IN
    );

    if (err < E_OK)
    {
        return err;
    }


    err = gpio_set_pin(
        PIN_ENCODER_LEFT_B,
        GPIO_MODE_IN
    );

    if (err < E_OK)
    {
        return err;
    }


    /* ============================================================
     * RIGHT ENCODER
     *
     * GP2 = Encoder A
     * GP3 = Encoder B
     * ============================================================
     */

    err = gpio_set_pin(
        PIN_ENCODER_RIGHT_A,
        GPIO_MODE_IN
    );

    if (err < E_OK)
    {
        return err;
    }


    err = gpio_set_pin(
        PIN_ENCODER_RIGHT_B,
        GPIO_MODE_IN
    );

    if (err < E_OK)
    {
        return err;
    }


    /*
     * Store the initial A-channel states.
     */
    left_a_last =
        gpio_get_val(
            PIN_ENCODER_LEFT_A
        );


    right_a_last =
        gpio_get_val(
            PIN_ENCODER_RIGHT_A
        );


    /*
     * Start encoder counters at zero.
     */
    encoder_reset();


    /*
     * Safety:
     *
     * make absolutely sure both motors are stopped.
     */
    motor_stop();


    /*
     * Motor commands may now be accepted.
     */
    motion_ready = TRUE;


    return E_OK;
}


/* ================================================================
 * MOTOR SPEED CONTROL
 * ================================================================
 */

void motor_set_left(int speed)
{
    if (!motion_ready)
    {
        return;
    }


    set_motor_pair(
        PIN_MOTOR_LEFT_A,
        PIN_MOTOR_LEFT_B,
        speed
    );
}


void motor_set_right(int speed)
{
    if (!motion_ready)
    {
        return;
    }


    set_motor_pair(
        PIN_MOTOR_RIGHT_A,
        PIN_MOTOR_RIGHT_B,
        speed
    );
}


void motor_set_speed(
    int left_speed,
    int right_speed
)
{
    motor_set_left(
        left_speed
    );

    motor_set_right(
        right_speed
    );
}


/* ================================================================
 * STOP
 * ================================================================
 */

void motor_stop(void)
{
    set_motor_pair(
        PIN_MOTOR_LEFT_A,
        PIN_MOTOR_LEFT_B,
        0
    );


    set_motor_pair(
        PIN_MOTOR_RIGHT_A,
        PIN_MOTOR_RIGHT_B,
        0
    );
}


/* ================================================================
 * SIMPLE DIRECTION HELPERS
 * ================================================================
 */

void motor_left_forward(void)
{
    motor_set_left(100);
}


void motor_left_reverse(void)
{
    motor_set_left(-100);
}


void motor_right_forward(void)
{
    motor_set_right(100);
}


void motor_right_reverse(void)
{
    motor_set_right(-100);
}


/* ================================================================
 * ENCODER POLLING
 * ================================================================
 */

void encoder_update(void)
{
    UINT left_a;
    UINT left_b;

    UINT right_a;
    UINT right_b;


    /*
     * Do nothing until motion hardware has been initialised.
     */
    if (!motion_ready)
    {
        return;
    }


    /*
     * Read both encoder channels.
     */
    left_a =
        gpio_get_val(
            PIN_ENCODER_LEFT_A
        );


    left_b =
        gpio_get_val(
            PIN_ENCODER_LEFT_B
        );


    right_a =
        gpio_get_val(
            PIN_ENCODER_RIGHT_A
        );


    right_b =
        gpio_get_val(
            PIN_ENCODER_RIGHT_B
        );


    /* ============================================================
     * LEFT ENCODER
     *
     * Count once whenever channel A changes:
     *
     *      LOW -> HIGH
     *
     * Channel B tells us the direction.
     * ============================================================
     */

    if (
        (left_a != 0U) &&
        (left_a_last == 0U)
    )
    {
        if (left_b != 0U)
        {
            left_encoder_count--;
        }
        else
        {
            left_encoder_count++;
        }
    }


    /* ============================================================
     * RIGHT ENCODER
     * ============================================================
     */

    if (
        (right_a != 0U) &&
        (right_a_last == 0U)
    )
    {
        if (right_b != 0U)
        {
            right_encoder_count--;
        }
        else
        {
            right_encoder_count++;
        }
    }


    /*
     * Save current A states for the next call.
     */
    left_a_last = left_a;
    right_a_last = right_a;
}


/* ================================================================
 * READ ENCODER COUNTS
 * ================================================================
 */

EncoderData encoder_get_counts(void)
{
    EncoderData data;


    data.left =
        left_encoder_count;


    data.right =
        right_encoder_count;


    return data;
}


/* ================================================================
 * RESET ENCODER COUNTS
 * ================================================================
 */

void encoder_reset(void)
{
    left_encoder_count = 0;
    right_encoder_count = 0;
}
>>>>>>> Stashed changes
