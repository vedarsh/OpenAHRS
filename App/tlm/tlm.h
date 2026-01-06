#ifndef TLM_H
#define TLM_H

#include "stm32f4xx.h"
#include "sensor_types.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= COMMAND NOUNS ================= */
#define GET_TLM_FUSED       0x01  // Get fused orientation (Euler angles)
#define GET_TLM_SENS        0x02  // Get calibrated sensor data
#define GET_TLM_RAW         0x03  // Get raw sensor data
#define GET_SYS_HEALTH      0x04  // Get system health status
#define GET_SYS_PARAMS      0x05  // Get system parameters
#define GET_AHRS_PARAMS     0x06  // Get AHRS parameters

#define SET_MAG_PARAMS      0x10  // Set magnetometer parameters
#define SET_IMU_PARAMS      0x11  // Set IMU parameters
#define SET_AHRS_PARAMS     0x12  // Set AHRS parameters

#define CMD_PING            0x20  // Ping command
#define CMD_RESET_COMMS     0x21  // Reset communications
#define CMD_SOFT_RESET      0x22  // Soft reset sensors
#define CMD_CALIB_START     0x23  // Start calibration mode

#define CMD_ERR             0xFF  // Error response

/* ================= MAGNETOMETER VERBS ================= */
#define MAG_SET_ODR         0x01  // Set output data rate
#define MAG_SET_MODE        0x02  // Set operating mode
#define MAG_ENABLE_COMP     0x03  // Enable temperature compensation
#define MAG_DISABLE_COMP    0x04  // Disable temperature compensation

/* ================= IMU VERBS ================= */
#define IMU_SET_ACCEL_FSR   0x01  // Set accelerometer full-scale range
#define IMU_SET_GYRO_FSR    0x02  // Set gyroscope full-scale range
#define IMU_SET_ODR         0x03  // Set output data rate

/* ================= AHRS VERBS ================= */
#define AHRS_SET_BETA       0x01  // Set filter gain
#define AHRS_RESET          0x02  // Reset quaternion

/* ================= RESPONSE CODES ================= */
#define RESP_ACK            0xAA  // Command acknowledged
#define RESP_NACK           0xBB  // Command not acknowledged
#define RESP_ERROR          0xCC  // Error occurred

/* ================= PACKET DEFINITIONS ================= */

// Fused orientation data (Euler angles + quaternion)
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF1
    float roll;
    float pitch;
    float yaw;
    float heading;
    float qw, qx, qy, qz;   // Quaternion
    uint8_t checksum;
    uint8_t end_byte;       // 0x1F
} tlm_fused_t;

// Calibrated sensor data
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF2
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
    float imu_temp_c;
    float mag_temp_c;
    uint8_t checksum;
    uint8_t end_byte;       // 0x2F
} tlm_sensor_t;

// Raw sensor data (uncalibrated)
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF3
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;
    int16_t imu_temp_raw;
    int16_t mag_temp_raw;
    uint8_t checksum;
    uint8_t end_byte;       // 0x3F
} tlm_raw_t;

// System health status
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF4
    sensor_state_t imu_health;
    sensor_state_t mag_health;
    uint32_t imu_error_count;
    uint32_t mag_error_count;
    uint32_t ahrs_update_count;
    uint32_t uptime_ms;
    uint8_t cpu_usage_percent;
    uint8_t checksum;
    uint8_t end_byte;       // 0x4F
} tlm_health_t;

// System parameters
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF5
    uint8_t imu_accel_fsr;
    uint8_t imu_gyro_fsr;
    uint8_t imu_odr;
    uint8_t mag_odr;
    uint8_t mag_mode;
    bool mag_temp_comp;
    uint16_t imu_sample_rate_hz;
    uint16_t mag_sample_rate_hz;
    uint8_t checksum;
    uint8_t end_byte;       // 0x5F
} tlm_sys_params_t;

// AHRS parameters
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xF6
    float beta;             // Filter gain
    float sample_period_s;
    bool use_magnetometer;
    uint8_t checksum;
    uint8_t end_byte;       // 0x6F
} tlm_ahrs_params_t;

// Simple response packet
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // RESP_ACK, RESP_NACK, or RESP_ERROR
    uint8_t command_echo;   // Echo back the command noun
    uint8_t status_code;    // Additional status information
    uint8_t end_byte;       // 0x55
} tlm_response_t;

// Legacy full packet (kept for compatibility)
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xAA
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    uint8_t date;
    uint8_t month;
    uint16_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    float roll;
    float pitch;
    float yaw;
    float heading_legacy;
    uint8_t end_byte;       // 0x55
} usb_packet_t;

/* ================= PUBLIC API ================= */
void tlm_init(void);
void process_tlm(void);

// Send telemetry packets
void tlm_send_fused(const tlm_fused_t *packet);
void tlm_send_sensor(const tlm_sensor_t *packet);
void tlm_send_raw(const tlm_raw_t *packet);
void tlm_send_health(const tlm_health_t *packet);
void tlm_send_sys_params(const tlm_sys_params_t *packet);
void tlm_send_ahrs_params(const tlm_ahrs_params_t *packet);
void tlm_send_response(uint8_t response_code, uint8_t command, uint8_t status);

#endif // TLM_H
