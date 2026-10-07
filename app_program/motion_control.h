#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <tk/tkernel.h>

typedef struct
{
    W left;
    W right;
} EncoderData;

/* Initialise motor PWM outputs and encoder GPIO inputs. */
ER motion_init(void);

/*
 * Motor speed range:
 *   -100 = full reverse
 *      0 = stop
 *   +100 = full forward
 */
void motor_set_left(int speed);
void motor_set_right(int speed);
void motor_set_speed(int left_speed, int right_speed);
void motor_stop(void);

/* Simple direction helpers retained for bring-up. */
void motor_left_forward(void);
void motor_left_reverse(void);
void motor_right_forward(void);
void motor_right_reverse(void);

/*
 * Bring-up encoder support.
 *
 * encoder_update() uses GPIO polling and must be called frequently.
 * Our app_main.c calls it every 2 ms.
 *
 * Buddy 2 can later replace this with GPIO interrupts for proper
 * high-speed odometry/PID without changing encoder_get_counts().
 */
void encoder_update(void);
EncoderData encoder_get_counts(void);
void encoder_reset(void);

#endif /* MOTION_CONTROL_H */