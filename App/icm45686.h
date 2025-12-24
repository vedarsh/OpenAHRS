/**
 * @file icm45686.h
 * @brief Driver for InvenSense ICM-45686 6-Axis IMU
 * @note Based on Datasheet DS-000577 Rev 1.0
 */

#ifndef ICM45686_H
#define ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "spi_types.h"

/** @brief Raw Sensor Data (Big Endian) */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} icm45686_raw_data_t;

/** @brief Engineering Units */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} icm45686_data_t;

/* =========================================================================
 *  Register Map (User Bank 0)
 * ========================================================================= */
// Data Registers (User Bank 0)
#define ICM45686_REG_ACCEL_DATA_X1      0x00
#define ICM45686_REG_ACCEL_DATA_X0      0x01
#define ICM45686_REG_TEMP_DATA1         0x0C
#define ICM45686_REG_PWR_MGMT0          0x10
#define ICM45686_REG_INT1_CONFIG0       0x16
#define ICM45686_REG_INT1_STATUS0       0x19
#define ICM45686_REG_ACCEL_CONFIG0      0x1B
#define ICM45686_REG_GYRO_CONFIG0       0x1C
#define ICM45686_REG_WHO_AM_I           0x72
#define ICM45686_REG_REGMISC2           0x7F

/* =========================================================================
 *  Bit Definitions & Constants
 * ========================================================================= */
#define ICM45686_CHIP_ID                0xE9 // Check your specific chip variation!

/* PWR_MGMT0 (0x10) */
#define ICM45686_PWR_GYRO_MODE_LN       (0x03 << 2) // Bit 3:2 = 11 (Low Noise)
#define ICM45686_PWR_ACCEL_MODE_LN      (0x03 << 0) // Bit 1:0 = 11 (Low Noise)

/* INT1_STATUS0 (0x19) */
#define ICM45686_INT_STATUS_DRDY        (1 << 2)    // Bit 2

/* REGMISC2 (0x7F) */
#define ICM45686_SOFT_RESET_BIT         (1 << 1)    // Bit 1

/* FSR Selections (Bits 7:5) */
// Note: Verify exact bit map with your specific datasheet table if different
#define ICM45686_FSR_32G    (0x00 << 5) // 000
#define ICM45686_FSR_16G    (0x01 << 5) // 001
#define ICM45686_FSR_8G     (0x02 << 5) // 010
#define ICM45686_FSR_4G     (0x03 << 5) // 011
#define ICM45686_FSR_2G     (0x04 << 5) // 100

#define ICM45686_FSR_4000DPS            (0x00 << 5)
#define ICM45686_FSR_2000DPS            (0x01 << 5)
#define ICM45686_FSR_1000DPS            (0x02 << 5)
#define ICM45686_FSR_500DPS             (0x03 << 5)

/* ODR Selections (Bits 3:0) */
#define ICM45686_ODR_32KHZ              0x01
#define ICM45686_ODR_16KHZ              0x02
#define ICM45686_ODR_8KHZ               0x03
#define ICM45686_ODR_4KHZ               0x04
#define ICM45686_ODR_2KHZ               0x05
#define ICM45686_ODR_1KHZ               0x06 // Standard default
#define ICM45686_ODR_200HZ              0x07
#define ICM45686_ODR_50HZ               0x08

/* =========================================================================
 *  Public API
 * ========================================================================= */
int32_t icm45686_read_chip_id(const spi_bus_t *bus);
int32_t icm45686_soft_reset(const spi_bus_t *bus);
int32_t icm45686_configure(const spi_bus_t *bus, uint8_t accel_fsr, uint8_t gyro_fsr, uint8_t odr);
int32_t icm45686_read_raw_data(const spi_bus_t *bus, icm45686_raw_data_t *data);
int32_t icm45686_read_sensor_ready(const spi_bus_t *bus);
void icm45686_convert_data(const icm45686_raw_data_t *raw, icm45686_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_H */
