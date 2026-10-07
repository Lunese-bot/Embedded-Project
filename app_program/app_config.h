#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#if !TM_CONSOLE_USB_CDC
#error "Robot baseline requires CONSOLE=usb_cdc because GP0/GP1 are used by the IMU I2C bus"
#endif

/*
 * Robot hardware pin map
 * ----------------------
 * Keep every application pin assignment in this one file so that all five
 * buddies work from the same map and do not accidentally claim the same GPIO.
 *
 * IMPORTANT: this baseline is intended to be built with CONSOLE=usb_cdc.
 * GP0/GP1 are then dedicated to I2C0 for the GY-511 LSM303DLHC.
 */

/* Buddy 4 - GY-511 / LSM303DLHC on I2C0 */
#define PIN_IMU_SDA                 0U
#define PIN_IMU_SCL                 1U

/* Buddy 2 - wheel encoders */
#define PIN_ENCODER_RIGHT_A         2U
#define PIN_ENCODER_RIGHT_B         3U
#define PIN_ENCODER_LEFT_A          4U
#define PIN_ENCODER_LEFT_B          5U

/* Buddy 3 - line sensors */
#define PIN_IR_LEFT_DIGITAL         6U
#define ADC_IR_CENTER_CHANNEL       0       /* GP26 / ADC0 */
#define ADC_IR_RIGHT_CHANNEL        1       /* GP27 / ADC1 */

/* Buddy 2 - Robo Pico onboard motor driver */
#define PIN_MOTOR_LEFT_A            8U      /* M1A */
#define PIN_MOTOR_LEFT_B            9U      /* M1B */
#define PIN_MOTOR_RIGHT_A          10U      /* M2A */
#define PIN_MOTOR_RIGHT_B          11U      /* M2B */

/* Buddy 5 - servo + ultrasonic */
#define PIN_SERVO                  12U
#define PIN_ULTRASONIC_ECHO        16U
#define PIN_ULTRASONIC_TRIG        17U

/* Bring-up test timing */
#define ROBOT_SENSOR_PERIOD_MS     500
#define ROBOT_IMU_PERIOD_MS        500
#define ROBOT_ENCODER_SAMPLE_MS      2

/*
 * Safety switches.
 * Leave both at 0 for normal sensor bring-up. Set to 1 only when the car is
 * lifted so the wheels/servo can move safely during the one-shot self-test.
 */
#define ROBOT_RUN_MOTOR_SELF_TEST    0
#define ROBOT_RUN_SERVO_SELF_TEST    0

/*
 * Motor PWM.
 * At 125 MHz and the reset-default divider of 1.0,
 * 12499 gives approximately 10 kHz.
 */
#define MOTOR_PWM_WRAP           12499U
#define MOTOR_SELF_TEST_SPEED       30

/* IMU */
#define IMU_I2C_DEVICE_NAME       "iica"
#define IMU_ACC_ADDR_PRIMARY       0x19U
#define IMU_ACC_ADDR_SECONDARY     0x18U

#endif /* APP_CONFIG_H */