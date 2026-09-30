#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

typedef enum
{
    LINE_STATE_INIT,
    LINE_STATE_CALIBRATE,
    LINE_STATE_SEARCH,
    LINE_STATE_FOLLOW,
    LINE_STATE_LOST,
    LINE_STATE_JUNCTION
} LineState;

typedef enum
{
    BARCODE_STATE_IDLE,
    BARCODE_STATE_DETECT,
    BARCODE_STATE_READ,
    BARCODE_STATE_DECODE,
    BARCODE_STATE_RETRY,
    BARCODE_STATE_COMPLETE
} BarcodeState;

typedef enum
{
    NAV_NONE,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_U_TURN
} NavigationCommand;

typedef struct
{
    int left;
    int centre;
    int right;
} IRReading;

#endif