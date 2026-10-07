<<<<<<< Updated upstream
=======
#ifndef IMU_H
#define IMU_H

#include <tk/tkernel.h>

typedef struct
{
    int x;
    int y;
    int z;
} IMU_AccelData;

/* Initialise/probe/configure the LSM303DLHC accelerometer. */
ER imu_init(void);

/* Preferred checked API. Returns E_OK only when a fresh sample was read. */
ER imu_read_accel(IMU_AccelData *accel);

/* Compatibility helper: returns the last valid sample (or zero before first). */
IMU_AccelData imu_get_accel(void);

BOOL imu_is_ready(void);
ER imu_get_last_error(void);
UW imu_get_error_count(void);
UW imu_get_address(void);

#endif /* IMU_H */
>>>>>>> Stashed changes
