#ifndef IMU_H
#define IMU_H

/* Raw accelerometer reading */
typedef struct {
    int x;
    int y;
    int z;
} IMU_AccelData;

/* Initialise the IMU module */
void imu_init(void);

/* Read the latest raw accelerometer values */
IMU_AccelData imu_get_accel(void);

#endif