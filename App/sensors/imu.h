#ifndef IMU_H
#define IMU_H

#include "stm32f4xx.h"
#include "icm45686.h"

#include "sensor_types.h"

#define DEFAULT_ACCEL_FSR ICM45686_FSR_8G
#define DEFAULT_GYRO_FSR  ICM45686_FSR_2000DPS
#define DEFAULT_ODR       ICM45686_ODR_1KHZ

typedef struct {
    uint8_t accel_fsr;
    uint8_t gyro_fsr;
    uint8_t odr;
} imu_config_setup_t;

typedef struct {
    imu_config_setup_t imu_config;
    spi_bus_t spi_bus;
    sensor_state_t health;
} imu_ctx_t;

imu_ctx_t* create_imu_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin);

sensor_state_t init_imu(imu_ctx_t *ctx);

#endif