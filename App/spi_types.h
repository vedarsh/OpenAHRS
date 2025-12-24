// spi_types.h
#ifndef SPI_TYPES_H
#define SPI_TYPES_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *nss_port;
    uint16_t           nss_pin;
} spi_bus_t;

#endif
