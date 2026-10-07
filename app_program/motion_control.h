// #ifndef MOTION_CONTROL_H
// #define MOTION_CONTROL_H

// typedef struct
// {
//     long left;
//     long right;
// } EncoderData;


// /* Initialise motors and encoders */
// void motion_init(void);


// /* Motor control
//  *
//  * speed range:
//  * -100 = full reverse
//  *    0 = stop
//  * +100 = full forward
//  */
// void motor_set_left(int speed);
// void motor_set_right(int speed);

// void motor_set_speed(int left_speed, int right_speed);

// void motor_stop(void);


// /* Encoder functions */
// EncoderData encoder_get_counts(void);

// void encoder_reset(void);

// #endif

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

void motion_init(void);

void motor_left_forward(void);
void motor_left_reverse(void);

void motor_right_forward(void);
void motor_right_reverse(void);

void motor_stop(void);

#endif