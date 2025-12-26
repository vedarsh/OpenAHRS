#ifndef SENSORS_H 
#define SENSORS_H

#include "stm32f4xx.h"
#include "lis2mdl.h"
#include "icm45686.h"
#include "main.h"


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

// sensors.h

typedef struct __attribute__((packed)) {
    uint8_t start_byte; // 0xAA
    
    // Raw Sensor Data (Inputs)
    float accel_x;
    float accel_y;
    float accel_z;
    
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    float mag_x;
    float mag_y;
    float mag_z;
    
    // AHRS Fused Data (Outputs)
    float roll;
    float pitch;
    float yaw;
    float heading_legacy;
    
    uint8_t end_byte;   // 0x55
} usb_packet_t;

typedef enum {
    SENSOR_NOT_INITIALISED = -3,
    SENSOR_OK = 0,
    SENSOR_DEGRADED = -1,
    SENSOR_ERROR = -2
} sensor_state_t;

bool app_init();
bool app_run();

#endif