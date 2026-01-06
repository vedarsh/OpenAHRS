#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include "spi_types.h"

typedef enum {
    SENSOR_NOT_INITIALISED = -3,
    SENSOR_OK = 0,
    SENSOR_DEGRADED = -1,
    SENSOR_ERROR = -2
} sensor_state_t;

#endif