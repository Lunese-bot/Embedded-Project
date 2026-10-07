#include <tk/tkernel.h>
#include "../device/include/dev_i2c.h"
#include "imu.h"

#define LSM303_ACC_ADDR_PRIMARY    0x19
#define LSM303_ACC_ADDR_SECONDARY  0x18

#define CTRL_REG1_A      0x20

#define OUT_X_L_A        0x28
#define OUT_X_H_A        0x29
#define OUT_Y_L_A        0x2A
#define OUT_Y_H_A        0x2B
#define OUT_Z_L_A        0x2C
#define OUT_Z_H_A        0x2D

static ID i2c_dd = -1;
static UW accel_addr = LSM303_ACC_ADDR_PRIMARY;
static BOOL imu_ready = FALSE;
static IMU_AccelData last_valid = {0, 0, 0};

static ER imu_try_configure(UW addr)
{
    ER err;
    UB verify = 0;

    err = i2c_write_reg(i2c_dd, addr, CTRL_REG1_A, 0x57);
    if (err < E_OK)
    {
        return err;
    }

    /* Read the register back so a failed I2C transaction is not
       mistaken for successful initialisation. */
    err = i2c_read_reg(i2c_dd, addr, CTRL_REG1_A, &verify);
    if (err < E_OK)
    {
        return err;
    }

    if (verify != 0x57)
    {
        return E_IO;
    }

    accel_addr = addr;
    return E_OK;
}

ER imu_init(void)
{
    ER err;

    imu_ready = FALSE;

    /* Open I2C0 device ("iica") once. */
    if (i2c_dd < E_OK)
    {
        i2c_dd = tk_opn_dev((UB *)"iica", TD_UPDATE);
        if (i2c_dd < E_OK)
        {
            return (ER)i2c_dd;
        }
    }

    /* Most LSM303DLHC boards use 0x19. Some use 0x18. */
    err = imu_try_configure(LSM303_ACC_ADDR_PRIMARY);
    if (err < E_OK)
    {
        err = imu_try_configure(LSM303_ACC_ADDR_SECONDARY);
    }

    if (err < E_OK)
    {
        return err;
    }

    imu_ready = TRUE;
    return E_OK;
}

static ER imu_read_reg_checked(UW reg, UB *value)
{
    ER err;

    err = i2c_read_reg(i2c_dd, accel_addr, reg, value);
    if (err < E_OK)
    {
        imu_ready = FALSE;
        return err;
    }

    return E_OK;
}

ER imu_read_accel(IMU_AccelData *accel)
{
    UB x_low = 0, x_high = 0;
    UB y_low = 0, y_high = 0;
    UB z_low = 0, z_high = 0;
    ER err;

    if (accel == NULL)
    {
        return E_PAR;
    }

    /* If an earlier transaction failed, try to recover automatically. */
    if (!imu_ready)
    {
        err = imu_init();
        if (err < E_OK)
        {
            return err;
        }
    }

    err = imu_read_reg_checked(OUT_X_L_A, &x_low);
    if (err < E_OK) return err;

    err = imu_read_reg_checked(OUT_X_H_A, &x_high);
    if (err < E_OK) return err;

    err = imu_read_reg_checked(OUT_Y_L_A, &y_low);
    if (err < E_OK) return err;

    err = imu_read_reg_checked(OUT_Y_H_A, &y_high);
    if (err < E_OK) return err;

    err = imu_read_reg_checked(OUT_Z_L_A, &z_low);
    if (err < E_OK) return err;

    err = imu_read_reg_checked(OUT_Z_H_A, &z_high);
    if (err < E_OK) return err;

    /* Preserve the same raw-value format used by your original code. */
    accel->x = (short)((x_high << 8) | x_low);
    accel->y = (short)((y_high << 8) | y_low);
    accel->z = (short)((z_high << 8) | z_low);

    last_valid = *accel;
    return E_OK;
}

IMU_AccelData imu_get_accel(void)
{
    IMU_AccelData accel = last_valid;

    /* Keep this function so any older code still compiles. */
    if (imu_read_accel(&accel) == E_OK)
    {
        return accel;
    }

    return last_valid;
}
