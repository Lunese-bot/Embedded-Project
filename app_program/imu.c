#include <tk/tkernel.h>
#include <stdint.h>

#include "../device/include/dev_i2c.h"
#include "app_config.h"
#include "imu.h"

#define CTRL_REG1_A      0x20U
#define CTRL_REG4_A      0x23U

#define OUT_X_L_A        0x28U
#define OUT_X_H_A        0x29U
#define OUT_Y_L_A        0x2AU
#define OUT_Y_H_A        0x2BU
#define OUT_Z_L_A        0x2CU
#define OUT_Z_H_A        0x2DU

static ID i2c_dd = -1;
static UW accel_addr = IMU_ACC_ADDR_PRIMARY;
static BOOL imu_ready = FALSE;
static ER last_error = E_IO;
static UW error_count = 0;
static IMU_AccelData last_valid = {0, 0, 0};

static ER read_reg(UW reg, UB *value)
{
    ER err;

    if ((i2c_dd < E_OK) || (value == NULL))
    {
        return E_IO;
    }

    err = i2c_read_reg(i2c_dd, accel_addr, reg, value);
    return err;
}

static ER write_reg(UW reg, UB value)
{
    if (i2c_dd < E_OK)
    {
        return E_IO;
    }

    return i2c_write_reg(i2c_dd, accel_addr, reg, value);
}

static ER probe_address(UW address)
{
    UB ctrl1 = 0;
    ER err;

    accel_addr = address;
    err = read_reg(CTRL_REG1_A, &ctrl1);

    /* A successful register transaction is enough to identify a responder. */
    return err;
}

ER imu_init(void)
{
    ER err_primary;
    ER err_secondary;
    ER err;
    UB verify = 0;

    imu_ready = FALSE;

    if (i2c_dd < E_OK)
    {
        i2c_dd = tk_opn_dev((UB *)IMU_I2C_DEVICE_NAME, TD_UPDATE);

        if (i2c_dd < E_OK)
        {
            last_error = (ER)i2c_dd;
            return last_error;
        }
    }

    /*
     * Most GY-511 boards use 0x19.
     * Some boards may use 0x18.
     */
    err_primary = probe_address(IMU_ACC_ADDR_PRIMARY);

    if (err_primary < E_OK)
    {
        err_secondary = probe_address(IMU_ACC_ADDR_SECONDARY);

        if (err_secondary < E_OK)
        {
            last_error = err_secondary;
            error_count++;
            return last_error;
        }
    }

    /*
     * CTRL_REG1_A = 0x57
     *
     * 100 Hz output data rate
     * normal operation
     * X/Y/Z enabled
     */
    err = write_reg(CTRL_REG1_A, 0x57U);

    if (err < E_OK)
    {
        last_error = err;
        error_count++;
        return err;
    }

    /*
     * CTRL_REG4_A = 0x88
     *
     * BDU enabled
     * +/-2 g
     * high-resolution mode
     */
    err = write_reg(CTRL_REG4_A, 0x88U);

    if (err < E_OK)
    {
        last_error = err;
        error_count++;
        return err;
    }

    /*
     * Read CTRL_REG1_A back.
     *
     * This confirms that the sensor actually accepted our configuration.
     * A broken I2C bus should not be treated as successful initialisation.
     */
    err = read_reg(CTRL_REG1_A, &verify);

    if ((err < E_OK) || (verify != 0x57U))
    {
        last_error = (err < E_OK) ? err : E_IO;
        error_count++;
        return last_error;
    }

    /*
     * Give the accelerometer enough time to generate its first sample.
     */
    tk_dly_tsk(20);

    imu_ready = TRUE;
    last_error = E_OK;

    return E_OK;
}

ER imu_read_accel(IMU_AccelData *accel)
{
    UB x_low = 0;
    UB x_high = 0;

    UB y_low = 0;
    UB y_high = 0;

    UB z_low = 0;
    UB z_high = 0;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    ER err;

    if (accel == NULL)
    {
        return E_PAR;
    }

    if (!imu_ready || (i2c_dd < E_OK))
    {
        return E_IO;
    }

#define READ_OR_FAIL(reg, ptr)                         \
    do                                                 \
    {                                                  \
        err = read_reg((reg), (ptr));                  \
                                                       \
        if (err < E_OK)                                \
        {                                              \
            last_error = err;                          \
            error_count++;                             \
            imu_ready = FALSE;                         \
            return err;                                \
        }                                              \
    } while (0)

    READ_OR_FAIL(OUT_X_L_A, &x_low);
    READ_OR_FAIL(OUT_X_H_A, &x_high);

    READ_OR_FAIL(OUT_Y_L_A, &y_low);
    READ_OR_FAIL(OUT_Y_H_A, &y_high);

    READ_OR_FAIL(OUT_Z_L_A, &z_low);
    READ_OR_FAIL(OUT_Z_H_A, &z_high);

#undef READ_OR_FAIL

    /*
     * Combine high byte and low byte.
     */
    raw_x =
        (int16_t)(((uint16_t)x_high << 8) |
                  (uint16_t)x_low);

    raw_y =
        (int16_t)(((uint16_t)y_high << 8) |
                  (uint16_t)y_low);

    raw_z =
        (int16_t)(((uint16_t)z_high << 8) |
                  (uint16_t)z_low);

    /*
     * LSM303DLHC high-resolution accelerometer data is
     * left-justified 12-bit data.
     *
     * Shift right by 4 bits to obtain the actual measurement.
     */
    accel->x = (int)(raw_x >> 4);
    accel->y = (int)(raw_y >> 4);
    accel->z = (int)(raw_z >> 4);

    /*
     * Save the last known-good measurement.
     */
    last_valid = *accel;

    last_error = E_OK;

    return E_OK;
}

IMU_AccelData imu_get_accel(void)
{
    IMU_AccelData sample = last_valid;

    if (imu_read_accel(&sample) == E_OK)
    {
        return sample;
    }

    /*
     * IMPORTANT:
     *
     * Never convert an I2C communication error into a new
     * fake measurement of:
     *
     * X=0
     * Y=0
     * Z=0
     *
     * Instead return the last valid reading.
     */
    return last_valid;
}

BOOL imu_is_ready(void)
{
    return imu_ready;
}

ER imu_get_last_error(void)
{
    return last_error;
}

UW imu_get_error_count(void)
{
    return error_count;
}

UW imu_get_address(void)
{
    return accel_addr;
}