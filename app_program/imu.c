#include <tk/tkernel.h>
#include "../device/include/dev_i2c.h"
#include "imu.h"

#define LSM303_ACC_ADDR  0x19

#define CTRL_REG1_A      0x20

#define OUT_X_L_A        0x28
#define OUT_X_H_A        0x29
#define OUT_Y_L_A        0x2A
#define OUT_Y_H_A        0x2B
#define OUT_Z_L_A        0x2C
#define OUT_Z_H_A        0x2D

static ID i2c_dd = -1;

void imu_init(void)
{
    ER err;
    /* Open I2C0 device ("iica") */
    i2c_dd = tk_opn_dev((UB *)"iica", TD_UPDATE);

    if (i2c_dd < E_OK)
    {
        /* Failed to open I2C device */
        return;
    }

    /*
     * Configure LSM303DLHC accelerometer.
     *
     * CTRL_REG1_A = 0x57
     *
     * 100 Hz output data rate
     * Normal mode
     * X, Y and Z axes enabled
     */
    err = i2c_write_reg(
        i2c_dd,
        LSM303_ACC_ADDR,
        CTRL_REG1_A,
        0x57
    );

    if (err < E_OK)
    {
        /* Failed to configure accelerometer */
        return;
    }
}

IMU_AccelData imu_get_accel(void)
{
    IMU_AccelData accel = {0, 0, 0};

    UB x_low = 0, x_high = 0;
    UB y_low = 0, y_high = 0;
    UB z_low = 0, z_high = 0;

    if (i2c_dd < E_OK)
    {
    return accel;
    }

    /* Read the 6 accelerometer registers */
    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_X_L_A, &x_low);
    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_X_H_A, &x_high);

    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_Y_L_A, &y_low);
    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_Y_H_A, &y_high);

    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_Z_L_A, &z_low);
    i2c_read_reg(i2c_dd, LSM303_ACC_ADDR, OUT_Z_H_A, &z_high);

    /* Combine the low and high bytes */
    accel.x = (short)((x_high << 8) | x_low);
    accel.y = (short)((y_high << 8) | y_low);
    accel.z = (short)((z_high << 8) | z_low);

    return accel;
}