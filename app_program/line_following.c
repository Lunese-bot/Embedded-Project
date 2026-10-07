#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "app_config.h"
#include "line_following.h"

static ID adc_dd = -1;

ER line_sensor_init(void)
{
    ER err;

    /* Left IR D0 -> digital input. */
    err = gpio_set_pin(PIN_IR_LEFT_DIGITAL, GPIO_MODE_IN);

    if (err < E_OK)
    {
        return err;
    }

    /*
     * GP26/GP27 are configured as analog inputs by the RP2040 startup code.
     * ADC unit 0 is registered as device "adca".
     */
    if (adc_dd < E_OK)
    {
        adc_dd = tk_opn_dev((UB *)"adca", TD_READ);
    }

    if (adc_dd < E_OK)
    {
        return (ER)adc_dd;
    }

    return E_OK;
}

int line_sensor_read_left(void)
{
    return (int)gpio_get_val(PIN_IR_LEFT_DIGITAL);
}

static ER read_adc_channel(W channel, uint16_t *value)
{
    UW raw = 0;
    SZ actual = 0;
    ER err;

    if (value == NULL)
    {
        return E_PAR;
    }

    /*
     * Start with zero, but we will also return an error code.
     *
     * This means the caller can distinguish:
     *
     * valid ADC value = 0
     *
     * from
     *
     * ADC communication/read failure.
     */
    *value = 0;

    if (adc_dd < E_OK)
    {
        return E_IO;
    }

    err = tk_srea_dev(
        adc_dd,
        channel,
        &raw,
        1,
        &actual
    );

    if (err < E_OK)
    {
        return err;
    }

    /*
     * We requested exactly one ADC sample.
     *
     * If the driver says it did not return one sample,
     * treat that as an error.
     */
    if (actual != 1)
    {
        return E_IO;
    }

    /*
     * RP2040 ADC is 12-bit.
     *
     * Keep only bits 0..11.
     */
    *value = (uint16_t)(raw & 0x0FFFU);

    return E_OK;
}

uint16_t line_sensor_read_center(void)
{
    uint16_t value = 0;

    /*
     * Compatibility helper.
     *
     * Existing project code can still call this function.
     * For the hardware test we use line_sensor_read_all()
     * because that version reports errors.
     */
    (void)read_adc_channel(
        ADC_IR_CENTER_CHANNEL,
        &value
    );

    return value;
}

uint16_t line_sensor_read_right(void)
{
    uint16_t value = 0;

    (void)read_adc_channel(
        ADC_IR_RIGHT_CHANNEL,
        &value
    );

    return value;
}

ER line_sensor_read_all(LineSensorData *data)
{
    ER err;

    if (data == NULL)
    {
        return E_PAR;
    }

    /*
     * Left IR sensor is digital.
     */
    data->left = line_sensor_read_left();

    /*
     * Centre IR sensor.
     */
    err = read_adc_channel(
        ADC_IR_CENTER_CHANNEL,
        &data->center
    );

    if (err < E_OK)
    {
        return err;
    }

    /*
     * Right IR sensor.
     */
    err = read_adc_channel(
        ADC_IR_RIGHT_CHANNEL,
        &data->right
    );

    if (err < E_OK)
    {
        return err;
    }

    return E_OK;
}

LineSensorData line_sensor_read(void)
{
    LineSensorData data = {0, 0, 0};

    /*
     * Keep this old API available so existing code does not break.
     *
     * For proper hardware testing, app_main.c uses:
     *
     * line_sensor_read_all()
     *
     * instead.
     */
    (void)line_sensor_read_all(&data);

    return data;
}