#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <tk/tkernel.h>

/* Configure GP16/17 ultrasonic pins and GP12 servo PWM. */
ER obstacle_init(void);

/* Returns distance in centimetres, or -1.0f on timeout/no echo. */
float ultrasonic_read_cm(void);

/* Basic servo command: angle is clamped to 0..180 degrees. */
void servo_set_angle(int angle);

#endif
