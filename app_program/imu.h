#ifndef IMU_H
#define IMU_H

#include <tk/tkernel.h>

/* Raw accelerometer reading */
typedef struct {
    int x;
    int y;
    int z;
} IMU_AccelData;

/* Initialise / configure the LSM303DLHC accelerometer. */
ER imu_init(void);

/* Checked read: E_OK means accel contains a fresh valid sample. */
ER imu_read_accel(IMU_AccelData *accel);

/* Compatibility helper for older code. */
IMU_AccelData imu_get_accel(void);

#endif
