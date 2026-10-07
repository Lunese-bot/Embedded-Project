#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "line_following.h"

#define IR_LEFT_PIN      6
#define IR_CENTER_ADC    0       /* GP26 = ADC0 */
#define IR_RIGHT_ADC     1       /* GP27 = ADC1 */

static ID adc_dd = -1;

ER line_sensor_init(void)
{
    ER err;

    /* Left IR D0 -> GP6 digital input. */
    err = gpio_set_pin(IR_LEFT_PIN, GPIO_MODE_IN);
    if (err < E_OK)
    {
        return err;
    }

    /*
     * The RP2040 startup code already initializes GP26/GP27 as analog pins
     * and registers ADC unit 0 as device "adca".
     */
    adc_dd = tk_opn_dev((UB *)"adca", TD_READ);
    if (adc_dd < E_OK)
    {
        return (ER)adc_dd;
    }

    return E_OK;
}

int line_sensor_read_left(void)
{
    return (int)gpio_get_val(IR_LEFT_PIN);
}

static uint16_t read_adc_channel(W channel)
{
    UW raw = 0;
    SZ actual = 0;
    ER err;

    if (adc_dd < E_OK)
    {
        return 0;
    }

    err = tk_srea_dev(adc_dd, channel, &raw, 1, &actual);

    if ((err < E_OK) || (actual != 1))
    {
        return 0;
    }

    return (uint16_t)(raw & 0x0FFFU);
}

uint16_t line_sensor_read_center(void)
{
    return read_adc_channel(IR_CENTER_ADC);
}

uint16_t line_sensor_read_right(void)
{
    return read_adc_channel(IR_RIGHT_ADC);
}

LineSensorData line_sensor_read(void)
{
    LineSensorData data;

    data.left = line_sensor_read_left();
    data.center = line_sensor_read_center();
    data.right = line_sensor_read_right();

    return data;
}
