#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"
#include "icm45686.h"
#include "sensor_types.h"  // Use your existing sensor_state_t

#define DEFAULT_ACCEL_FSR ICM45686_FSR_8G
#define DEFAULT_GYRO_FSR  ICM45686_FSR_2000DPS
#define DEFAULT_ODR       ICM45686_ODR_1KHZ

// Renamed to avoid conflict with app.h
typedef struct {
    uint8_t accel_fsr;
    uint8_t gyro_fsr;
    uint8_t odr;
} imu_config_t;

typedef struct {
    imu_config_t imu_config;
    spi_bus_t spi_bus;
    sensor_state_t health;
} imu_ctx_t;

typedef struct {
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    float gyr_x_dps;
    float gyr_y_dps;
    float gyr_z_dps;
    
    float temp_c;
} imu_output_t;

// Public API
imu_ctx_t* create_imu_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin);
sensor_state_t init_imu(imu_ctx_t *ctx);
sensor_state_t imu_reconfigure(imu_ctx_t *ctx);
sensor_state_t imu_read_data(imu_ctx_t *ctx, imu_output_t *output);
sensor_state_t imu_data_ready(imu_ctx_t *ctx);

#endif // IMU_H
