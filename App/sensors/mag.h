#ifndef MAG_H
#define MAG_H

#include "stm32f4xx_hal.h"
#include "lis2mdl.h"
#include "sensor_types.h"

#define DEFAULT_MAG_ODR         LIS2MDL_ODR_100HZ
#define DEFAULT_MAG_MODE        LIS2MDL_MODE_CONTINUOUS

typedef struct {
    lis2mdl_odr_t odr;
    lis2mdl_mode_t mode;
    bool temp_comp_enabled;
    bool low_pass_filter_enabled;
} mag_config_t;

typedef struct {
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
    float temp_c;
} mag_output_t;

typedef struct {
    spi_bus_t spi_bus;
    mag_config_t mag_config;
    sensor_state_t health;
} mag_ctx_t;

// Public API
mag_ctx_t* create_mag_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin);
sensor_state_t init_mag(mag_ctx_t *ctx);
sensor_state_t mag_reconfigure(mag_ctx_t *ctx);
sensor_state_t mag_read_data(mag_ctx_t *ctx, mag_output_t *output);
sensor_state_t mag_data_ready(mag_ctx_t *ctx);

#endif // MAG_H
