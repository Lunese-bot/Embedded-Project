<<<<<<< Updated upstream
=======
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <tk/tkernel.h>

/* Configure ultrasonic pins and servo PWM. */
ER obstacle_init(void);

/* Returns distance in centimetres, or -1.0f on timeout/no echo. */
float ultrasonic_read_cm(void);

/* Angle is clamped to 0..180 degrees. */
ER servo_set_angle(int angle);

#endif /* OBSTACLE_H */
>>>>>>> Stashed changes
