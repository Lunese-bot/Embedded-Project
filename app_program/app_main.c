/*
 * Robot hardware bring-up baseline
 *
 * Purpose:
 * Before the five buddies begin implementing their individual project
 * algorithms, this program verifies that the common robot hardware works
 * reliably under micro T-Kernel.
 *
 * Hardware tested here:
 *
 * Buddy 2:
 *   - Left/right motor driver initialisation
 *   - Left/right wheel encoders
 *
 * Buddy 3:
 *   - Left/centre/right IR sensors
 *
 * Buddy 4:
 *   - GY-511 LSM303DLHC accelerometer
 *
 * Buddy 5:
 *   - Ultrasonic distance sensor
 *   - Servo initialisation
 *
 * Motors are kept STOPPED unless ROBOT_RUN_MOTOR_SELF_TEST is enabled
 * in app_config.h.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "demo_tasks.h"

#include "app_config.h"
#include "imu.h"
#include "line_following.h"
#include "obstacle.h"
#include "motion_control.h"


/*
 * Stack size for each robot test task.
 */
#define STACK_SZ 4096


/*
 * Set TRUE after motion_control has successfully initialised.
 *
 * The optional motor test waits for this before attempting
 * to move the motors.
 */
static volatile BOOL motion_ready = FALSE;


/* ================================================================
 * USB SERIAL MONITOR WAIT
 * ================================================================
 *
 * The Pico can start executing tasks before Windows has finished
 * detecting the USB serial connection.
 *
 * Waiting briefly here makes it less likely that the first status
 * messages disappear before Serial Monitor connects.
 */
static void wait_for_console(void)
{
    INT i;

    for (i = 0; i < 100 && tm_usb_state() < 2; i++)
    {
        tk_dly_tsk(50);
    }
}


/* ================================================================
 * MOTION / ENCODER TASK
 * ================================================================
 *
 * Buddy 2 hardware baseline.
 *
 * This task:
 *   1. Initialises the motor driver.
 *   2. Ensures the motors begin stopped.
 *   3. Continuously updates encoder counts.
 *
 * It DOES NOT make the robot drive automatically.
 */
LOCAL void motion_io_task(INT stacd, void *exinf)
{
    ER err;

    (void)stacd;
    (void)exinf;

    wait_for_console();

    /*
     * Keep trying until motor/encoder initialisation succeeds.
     */
    do
    {
        err = motion_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[MOTION] init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    motion_ready = TRUE;

    tm_printf(
        (UB *)"[MOTION] ready: motors stopped, encoder polling active\n"
    );


    /*
     * Continuously sample the wheel encoders.
     */
    while (1)
    {
        encoder_update();

        tk_dly_tsk(ROBOT_ENCODER_SAMPLE_MS);
    }
}


/* ================================================================
 * IR + ULTRASONIC + SERVO TASK
 * ================================================================
 *
 * Buddy 3 and Buddy 5 hardware baseline.
 *
 * This task continuously displays:
 *
 *   IR Left
 *   IR Centre
 *   IR Right
 *   Ultrasonic distance
 *   Left encoder count
 *   Right encoder count
 */
LOCAL void sensor_task(INT stacd, void *exinf)
{
    LineSensorData line = {0, 0, 0};

    EncoderData enc;

    float distance;

    ER line_err;
    ER err;


    (void)stacd;
    (void)exinf;


    wait_for_console();


    /* ------------------------------------------------------------
     * Initialise IR sensors
     * ------------------------------------------------------------
     */

    do
    {
        err = line_sensor_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[SENSOR] line_sensor_init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    /* ------------------------------------------------------------
     * Initialise ultrasonic + servo
     * ------------------------------------------------------------
     */

    do
    {
        err = obstacle_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[SENSOR] obstacle_init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    /* ------------------------------------------------------------
     * OPTIONAL SERVO SELF TEST
     *
     * Disabled by default in app_config.h.
     * ------------------------------------------------------------
     */

#if ROBOT_RUN_SERVO_SELF_TEST

    tm_printf(
        (UB *)"[SERVO] one-shot test: 45 -> 90 -> 135 -> 90 deg\n"
    );

    (void)servo_set_angle(45);

    tk_dly_tsk(700);


    (void)servo_set_angle(90);

    tk_dly_tsk(700);


    (void)servo_set_angle(135);

    tk_dly_tsk(700);


    (void)servo_set_angle(90);

#endif


    tm_printf(
        (UB *)"[SENSOR] ready: IR + ultrasonic + servo centre\n"
    );


    /* ------------------------------------------------------------
     * Continuous hardware readings
     * ------------------------------------------------------------
     */

    while (1)
    {
        /*
         * Read all three IR sensors.
         */
        line_err = line_sensor_read_all(&line);


        /*
         * Measure ultrasonic distance.
         */
        distance = ultrasonic_read_cm();


        /*
         * Obtain latest encoder counts.
         */
        enc = encoder_get_counts();


        /*
         * IR failed.
         */
        if (line_err < E_OK)
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR read error=%d | ENC L=%d R=%d\n",

                line_err,
                enc.left,
                enc.right
            );
        }

        /*
         * Ultrasonic timed out.
         */
        else if (distance < 0.0f)
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR L=%d C=%u R=%u | "
                "US=TIMEOUT | ENC L=%d R=%d\n",

                line.left,
                (UINT)line.center,
                (UINT)line.right,
                enc.left,
                enc.right
            );
        }

        /*
         * Normal sensor readings.
         */
        else
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR L=%d C=%u R=%u | "
                "US=%d cm | ENC L=%d R=%d\n",

                line.left,
                (UINT)line.center,
                (UINT)line.right,
                (INT)distance,
                enc.left,
                enc.right
            );
        }


        tk_dly_tsk(ROBOT_SENSOR_PERIOD_MS);
    }
}


/* ================================================================
 * IMU TASK
 * ================================================================
 *
 * Buddy 4 hardware baseline.
 *
 * Important difference from your old code:
 *
 * We DO NOT automatically print
 *
 *      X=0 Y=0 Z=0
 *
 * when the I2C communication fails.
 *
 * The return code from the IMU driver is checked.
 */
LOCAL void imu_task(INT stacd, void *exinf)
{
    IMU_AccelData accel;

    ER err;

    UW shown_addr = 0xFFFFFFFFU;


    (void)stacd;
    (void)exinf;


    wait_for_console();


    while (1)
    {
        /* --------------------------------------------------------
         * IMU not connected/initialised
         * --------------------------------------------------------
         */

        if (!imu_is_ready())
        {
            err = imu_init();


            if (err < E_OK)
            {
                tm_printf(
                    (UB *)
                    "[IMU] init/I2C failed err=%d "
                    "(GP0=SDA GP1=SCL); retrying\n",

                    err
                );


                tk_dly_tsk(1000);

                continue;
            }


            /*
             * Print the detected accelerometer address.
             *
             * Usually:
             *
             * 0x19
             *
             * Some boards may use:
             *
             * 0x18
             */
            if (shown_addr != imu_get_address())
            {
                shown_addr = imu_get_address();

                tm_printf(
                    (UB *)
                    "[IMU] ready on I2C address 0x%02x\n",

                    shown_addr
                );
            }
        }


        /* --------------------------------------------------------
         * Read accelerometer
         * --------------------------------------------------------
         */

        err = imu_read_accel(&accel);


        /*
         * VERY IMPORTANT:
         *
         * An I2C error is reported as an ERROR.
         *
         * It is no longer displayed as fake:
         *
         * X=0 Y=0 Z=0
         */
        if (err < E_OK)
        {
            tm_printf(
                (UB *)
                "[IMU] read failed err=%d "
                "total_errors=%u; reconnecting\n",

                err,
                imu_get_error_count()
            );


            tk_dly_tsk(500);

            continue;
        }


        /* --------------------------------------------------------
         * Successful accelerometer reading
         * --------------------------------------------------------
         */

        tm_printf(
            (UB *)
            "[IMU] X=%d Y=%d Z=%d\n",

            accel.x,
            accel.y,
            accel.z
        );


        tk_dly_tsk(ROBOT_IMU_PERIOD_MS);
    }
}


/* ================================================================
 * OPTIONAL MOTOR SELF TEST
 * ================================================================
 *
 * This entire task is excluded when:
 *
 * ROBOT_RUN_MOTOR_SELF_TEST == 0
 *
 * Keep it disabled during our first test.
 */

#if ROBOT_RUN_MOTOR_SELF_TEST

LOCAL void motor_self_test_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;


    wait_for_console();


    /*
     * Wait until the main motion task has initialised the motor driver.
     */
    while (!motion_ready)
    {
        tk_dly_tsk(50);
    }


    tm_printf(
        (UB *)
        "[MOTOR TEST] starting in 3 seconds - "
        "keep wheels off ground\n"
    );


    tk_dly_tsk(3000);


    /* ------------------------------------------------------------
     * LEFT FORWARD
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] left forward %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_left(MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * RIGHT FORWARD
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] right forward %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_right(MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * LEFT REVERSE
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] left reverse %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_left(-MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * RIGHT REVERSE
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] right reverse %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_right(-MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();


    tm_printf(
        (UB *)
        "[MOTOR TEST] finished; motors stopped\n"
    );


    /*
     * Test only runs once.
     */
    tk_slp_tsk(TMO_FEVR);
}

#endif


/* ================================================================
 * RTOS TASK CONFIGURATION
 * ================================================================
 */


/*
 * Motor + encoder task
 */
LOCAL T_CTSK ctsk_motion_io =
{
    .itskpri = 6,
    .stksz   = STACK_SZ,
    .task    = motion_io_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * IMU task
 */
LOCAL T_CTSK ctsk_imu =
{
    .itskpri = 8,
    .stksz   = STACK_SZ,
    .task    = imu_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * IR + ultrasonic + encoder display
 */
LOCAL T_CTSK ctsk_sensor =
{
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = sensor_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * Existing Pico LED heartbeat task
 */
LOCAL T_CTSK ctsk_blink =
{
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


#if ROBOT_RUN_MOTOR_SELF_TEST

LOCAL T_CTSK ctsk_motor_test =
{
    .itskpri = 7,
    .stksz   = STACK_SZ,
    .task    = motor_self_test_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

#endif


/* ================================================================
 * TASK STARTING HELPER
 * ================================================================
 */

static BOOL start_task(const char *name, T_CTSK *cfg)
{
    ID tid;

    ER err;


    /*
     * Create task.
     */
    tid = tk_cre_tsk(cfg);


    if (tid <= E_OK)
    {
        tm_printf(
            (UB *)
            "[INIT] task %s create failed err=%d\n",

            name,
            tid
        );


        return FALSE;
    }


    /*
     * Start task.
     */
    err = tk_sta_tsk(tid, 0);


    if (err < E_OK)
    {
        tm_printf(
            (UB *)
            "[INIT] task %s start failed err=%d\n",

            name,
            err
        );


        return FALSE;
    }


    return TRUE;
}


/* ================================================================
 * APPLICATION ENTRY POINT
 * ================================================================
 */

EXPORT INT usermain(void)
{
    /*
     * Start Pico heartbeat LED.
     */
    (void)start_task(
        "blink",
        &ctsk_blink
    );


    /*
     * Start motor/encoder hardware task.
     */
    (void)start_task(
        "motion_io",
        &ctsk_motion_io
    );


    /*
     * Start IMU task.
     */
    (void)start_task(
        "imu",
        &ctsk_imu
    );


    /*
     * Start IR / ultrasonic / encoder display task.
     */
    (void)start_task(
        "sensor",
        &ctsk_sensor
    );


    /*
     * Optional motor movement test.
     *
     * It will not be compiled when
     * ROBOT_RUN_MOTOR_SELF_TEST = 0.
     */
#if ROBOT_RUN_MOTOR_SELF_TEST

    (void)start_task(
        "motor_test",
        &ctsk_motor_test
    );

#endif


    /*
     * VERY IMPORTANT:
     *
     * usermain must not return.
     *
     * All worker tasks have already been started above,
     * so the initial application task can now sleep forever.
     *
     * Unlike your previous app_main.c, there is NOTHING after
     * this sleep that still needs to execute.
     */
    tk_slp_tsk(TMO_FEVR);


    return 0;
}