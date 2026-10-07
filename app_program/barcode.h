#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

void barcode_init(void);

/* Give barcode decoder the newest IR reading */
void barcode_update(uint16_t sensor_value);


/* Returns true when a complete barcode has been decoded */
bool barcode_available(void);


/* Returns A, B, C, D etc. */
char barcode_get(void);

#endif