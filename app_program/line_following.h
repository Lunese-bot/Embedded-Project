<<<<<<< Updated upstream
=======
#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <tk/tkernel.h>
#include <stdint.h>

typedef struct
{
    int left;           /* GP6 digital: 0 or 1 */
    uint16_t center;    /* GP26 / ADC0: 0..4095 */
    uint16_t right;     /* GP27 / ADC1: 0..4095 */
} LineSensorData;

/* Open/configure the line sensors. Returns E_OK on success. */
ER line_sensor_init(void);

/* Checked API for the hardware baseline. */
ER line_sensor_read_all(LineSensorData *data);

/* Existing convenience functions. */
int line_sensor_read_left(void);
uint16_t line_sensor_read_center(void);
uint16_t line_sensor_read_right(void);
LineSensorData line_sensor_read(void);

#endif /* LINE_FOLLOWING_H */
>>>>>>> Stashed changes
