#ifndef APP_H 
#define APP_H

#include "stm32f4xx.h"
#include "lis2mdl.h"
#include "icm45686.h"
#include "ds3231.h"
#include "main.h"

#define TLM_DEFAULT_HEADER 0xAA
#define TLM_DEFAULT_TAIL  0x55

typedef struct {
    bool spi_4_mode;
    bool bdu;
    bool disable_i2c;
    bool enable_compensation;
    bool enable_lpf;
    lis2mdl_odr_t odr_status;
    lis2mdl_mode_t mode_status;
} mag_config_setup_t;

typedef struct {
    uint8_t accel_fsr;
    uint8_t gyro_fsr;
    uint8_t odr;
} imu_config_setup_t;

typedef struct {
    lis2mdl_data_t mag_data_ut;
    float temp_c;
    bool is_data_stale;
} mag_data_out_t;

typedef struct {
    icm45686_data_t imu_data;
    bool is_data_stale;
} imu_data_out_t;

bool app_init();
bool app_run();

#endif