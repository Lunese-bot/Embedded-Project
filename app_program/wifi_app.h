#ifndef WIFI_APP_H
#define WIFI_APP_H

#include <stdbool.h>

typedef struct
{
    float distance_cm;

    int accel_x;
    int accel_y;
    int accel_z;

    long left_encoder;
    long right_encoder;

    char barcode;

} TelemetryData;


/* Initialise WiFi application */
void wifi_app_init(void);


/* Send current robot information */
bool wifi_send_telemetry(TelemetryData *data);


/* Check for incoming command */
bool wifi_command_available(void);


/* Get latest command */
char wifi_get_command(void);


#endif