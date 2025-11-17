/**
 * @file icm45686.h
 * @brief ICM-45686 6-Axis IMU (Accelerometer + Gyroscope) SPI Driver
 *
 * Hardware: TDK InvenSense ICM-45686 3-axis accelerometer + 3-axis gyroscope
 * Interface: SPI (4-wire mode, max 24 MHz)
 * Platform: Hardware-independent via SPI bus abstraction
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 17, 2025
 *
 * @note This driver follows NASA JPL and MISRA C coding standards
 * @note All register addresses from ICM-45686 datasheet DS-000577 Rev 1.0
 */

#ifndef ICM45686_H
#define ICM45686_H

#include "spi_bus.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * PREPROCESSOR DEFINITIONS
 * ============================================================================ */

/** @defgroup ICM45686_Constants Device Constants */
/** @{ */

/* Device Identification */
#define ICM45686_WHO_AM_I_REG       (0x72U)    /**< WHO_AM_I register address */
#define ICM45686_WHO_AM_I_VALUE     (0xE9U)    /**< Expected WHO_AM_I response */

/* Timing Constants (from datasheet) */
#define ICM45686_STARTUP_TIME_MS    (50U)      /**< Device startup time after power-on */
#define ICM45686_SOFT_RESET_TIME_MS (10U)      /**< Software reset settling time */
#define ICM45686_MODE_SWITCH_TIME_MS (100U)    /**< Power mode switch stabilization time */

/** @} */

/** @defgroup ICM45686_Register_Map Register Addresses (User Bank 0) */
/** @{ */

/* Sensor Data Registers (Big Endian) */
#define ICM45686_REG_ACCEL_DATA_X1_UI   (0x00U)  /**< Accel X-axis data [15:8] */
#define ICM45686_REG_ACCEL_DATA_X0_UI   (0x01U)  /**< Accel X-axis data [7:0] */
#define ICM45686_REG_ACCEL_DATA_Y1_UI   (0x02U)  /**< Accel Y-axis data [15:8] */
#define ICM45686_REG_ACCEL_DATA_Y0_UI   (0x03U)  /**< Accel Y-axis data [7:0] */
#define ICM45686_REG_ACCEL_DATA_Z1_UI   (0x04U)  /**< Accel Z-axis data [15:8] */
#define ICM45686_REG_ACCEL_DATA_Z0_UI   (0x05U)  /**< Accel Z-axis data [7:0] */

#define ICM45686_REG_GYRO_DATA_X1_UI    (0x06U)  /**< Gyro X-axis data [15:8] */
#define ICM45686_REG_GYRO_DATA_X0_UI    (0x07U)  /**< Gyro X-axis data [7:0] */
#define ICM45686_REG_GYRO_DATA_Y1_UI    (0x08U)  /**< Gyro Y-axis data [15:8] */
#define ICM45686_REG_GYRO_DATA_Y0_UI    (0x09U)  /**< Gyro Y-axis data [7:0] */
#define ICM45686_REG_GYRO_DATA_Z1_UI    (0x0AU)  /**< Gyro Z-axis data [15:8] */
#define ICM45686_REG_GYRO_DATA_Z0_UI    (0x0BU)  /**< Gyro Z-axis data [7:0] */

#define ICM45686_REG_TEMP_DATA1_UI      (0x0CU)  /**< Temperature data [15:8] */
#define ICM45686_REG_TEMP_DATA0_UI      (0x0DU)  /**< Temperature data [7:0] */

/* Configuration and Control Registers */
#define ICM45686_REG_PWR_MGMT0          (0x10U)  /**< Power management register */
#define ICM45686_REG_ACCEL_CONFIG0      (0x1BU)  /**< Accelerometer configuration */
#define ICM45686_REG_GYRO_CONFIG0       (0x1CU)  /**< Gyroscope configuration */
#define ICM45686_REG_FIFO_CONFIG0       (0x1DU)  /**< FIFO configuration register 0 */
#define ICM45686_REG_FIFO_CONFIG3       (0x21U)  /**< FIFO configuration register 3 */

#define ICM45686_REG_INT1_CONFIG0       (0x16U)  /**< Interrupt 1 configuration 0 */
#define ICM45686_REG_INT1_CONFIG1       (0x17U)  /**< Interrupt 1 configuration 1 */
#define ICM45686_REG_INT1_CONFIG2       (0x18U)  /**< Interrupt 1 configuration 2 */
#define ICM45686_REG_INT1_STATUS0       (0x19U)  /**< Interrupt 1 status flags 0 */

#define ICM45686_REG_INTF_CONFIG0       (0x2CU)  /**< Interface configuration 0 */
#define ICM45686_REG_DRIVE_CONFIG0      (0x32U)  /**< Drive configuration (slew rate) */

/** @} */

/** @defgroup ICM45686_Power_Modes Power Management Modes */
/** @{ */

/* PWR_MGMT0 Register Bit Definitions */
#define ICM45686_ACCEL_MODE_OFF         (0x00U)  /**< Accelerometer off */
#define ICM45686_ACCEL_MODE_LP          (0x02U)  /**< Accelerometer low-power mode */
#define ICM45686_ACCEL_MODE_LN          (0x03U)  /**< Accelerometer low-noise mode */

#define ICM45686_GYRO_MODE_OFF          (0x00U)  /**< Gyroscope off */
#define ICM45686_GYRO_MODE_LP           (0x08U)  /**< Gyroscope low-power mode */
#define ICM45686_GYRO_MODE_LN           (0x0CU)  /**< Gyroscope low-noise mode */

/** @} */

/** @defgroup ICM45686_FSR Full Scale Range Settings */
/** @{ */

/* Gyroscope Full-Scale Range (GYRO_UI_FS_SEL bits [7:4]) */
#define ICM45686_GYRO_FS_4000DPS    (0x00U)    /**< ±4000 dps, 8.2 LSB/dps */
#define ICM45686_GYRO_FS_2000DPS    (0x10U)    /**< ±2000 dps, 16.4 LSB/dps */
#define ICM45686_GYRO_FS_1000DPS    (0x20U)    /**< ±1000 dps, 32.8 LSB/dps */
#define ICM45686_GYRO_FS_500DPS     (0x30U)    /**< ±500 dps, 65.5 LSB/dps */
#define ICM45686_GYRO_FS_250DPS     (0x40U)    /**< ±250 dps, 131 LSB/dps */
#define ICM45686_GYRO_FS_125DPS     (0x50U)    /**< ±125 dps, 262 LSB/dps */
#define ICM45686_GYRO_FS_62_5DPS    (0x60U)    /**< ±62.5 dps, 524.3 LSB/dps */
#define ICM45686_GYRO_FS_31_25DPS   (0x70U)    /**< ±31.25 dps, 1048.6 LSB/dps */
#define ICM45686_GYRO_FS_15_625DPS  (0x80U)    /**< ±15.625 dps, 2097.2 LSB/dps */

/* Accelerometer Full-Scale Range (ACCEL_UI_FS_SEL bits [6:4]) */
#define ICM45686_ACCEL_FS_32G       (0x00U << 4)    /**< ±32g, 1024 LSB/g */
#define ICM45686_ACCEL_FS_16G       (0x01U << 4)    /**< ±16g, 2048 LSB/g */
#define ICM45686_ACCEL_FS_8G        (0x02U << 4)    /**< ±8g, 4096 LSB/g */
#define ICM45686_ACCEL_FS_4G        (0x03U << 4)    /**< ±4g, 8192 LSB/g */
#define ICM45686_ACCEL_FS_2G        (0x04U << 4)    /**< ±2g, 16384 LSB/g */

/** @} */

/** @defgroup ICM45686_Interrupt_Config Interrupt Configuration */
/** @{ */

/* INT1_CONFIG0 Register Bit Masks */
#define ICM45686_INT1_CFG0_RESET_DONE_EN_MASK   (0x80U)
#define ICM45686_INT1_CFG0_AUX1_AGC_RDY_EN_MASK (0x40U)
#define ICM45686_INT1_CFG0_AP_AGC_RDY_EN_MASK   (0x20U)
#define ICM45686_INT1_CFG0_AP_FSYNC_EN_MASK     (0x10U)
#define ICM45686_INT1_CFG0_AUX1_DRDY_EN_MASK    (0x08U)
#define ICM45686_INT1_CFG0_DRDY_EN_MASK         (0x04U)
#define ICM45686_INT1_CFG0_FIFO_THS_EN_MASK     (0x02U)
#define ICM45686_INT1_CFG0_FIFO_FULL_EN_MASK    (0x01U)

/** @} */

/** @defgroup ICM45686_ODR Output Data Rate Settings */
/** @{ */

#define ICM45686_ODR_32KHZ              (0x01U)  /**< 32 kHz ODR (LN mode only) */
#define ICM45686_ODR_16KHZ              (0x02U)  /**< 16 kHz ODR (LN mode only) */
#define ICM45686_ODR_8KHZ               (0x03U)  /**< 8 kHz ODR */
#define ICM45686_ODR_4KHZ               (0x04U)  /**< 4 kHz ODR */
#define ICM45686_ODR_2KHZ               (0x05U)  /**< 2 kHz ODR */
#define ICM45686_ODR_1KHZ               (0x06U)  /**< 1 kHz ODR */
#define ICM45686_ODR_500HZ              (0x0FU)  /**< 500 Hz ODR */
#define ICM45686_ODR_200HZ              (0x07U)  /**< 200 Hz ODR */
#define ICM45686_ODR_100HZ              (0x08U)  /**< 100 Hz ODR */
#define ICM45686_ODR_50HZ               (0x09U)  /**< 50 Hz ODR */

/** @} */

/** @defgroup ICM45686_Error_Codes Driver Error Codes */
/** @{ */

#define ICM45686_OK                     (0)      /**< Operation successful */
#define ICM45686_ERR_SPI                (-1)     /**< SPI communication error */
#define ICM45686_ERR_TIMEOUT            (-2)     /**< Operation timeout */
#define ICM45686_ERR_INVALID_PARAM      (-3)     /**< Invalid parameter */
#define ICM45686_ERR_WHO_AM_I           (-4)     /**< WHO_AM_I mismatch */
#define ICM45686_ERR_NOT_INITIALIZED    (-5)     /**< Device not initialized */
#define ICM45686_ERR_RANGE              (-6)     /**< Data out of valid range */

/** @} */

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

/**
 * @brief IMU raw sensor data structure
 */
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;
} Icm45686_RawData_t;

/**
 * @brief IMU scaled sensor data structure
 */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_degc;
} Icm45686_ScaledData_t;

/**
 * @brief Device configuration structure
 */
typedef struct {
    uint8_t accel_fsr;
    uint8_t gyro_fsr;
    uint8_t accel_odr;
    uint8_t gyro_odr;
    uint8_t accel_mode;
    uint8_t gyro_mode;
    uint8_t drdy_mode;
} Icm45686_Config_t;

/**
 * @brief Device handle structure
 */
typedef struct {
    Spi_Device_t spi_device;            /**< SPI device handle (bus abstraction) */
    Icm45686_Config_t config;           /**< Device configuration */
    bool is_initialized;                /**< Initialization status flag */
    float accel_scale_factor;           /**< Accelerometer LSB to g conversion */
    float gyro_scale_factor;            /**< Gyroscope LSB to dps conversion */
} Icm45686_Handle_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

int32_t icm45686_init(Icm45686_Handle_t* handle,
                      SPI_HandleTypeDef* hspi,
                      GPIO_TypeDef* cs_port,
                      uint16_t cs_pin,
                      const Icm45686_Config_t* config);

int32_t icm45686_read_who_am_i(Icm45686_Handle_t* handle, uint8_t* who_am_i);

int32_t icm45686_read_raw_data(Icm45686_Handle_t* handle,
                               Icm45686_RawData_t* data);

int32_t icm45686_read_scaled_data(Icm45686_Handle_t* handle,
                                  Icm45686_ScaledData_t* data);

int32_t icm45686_soft_reset(Icm45686_Handle_t* handle);

int32_t icm45686_set_accel_fsr(Icm45686_Handle_t* handle, uint8_t fsr);
int32_t icm45686_set_gyro_fsr(Icm45686_Handle_t* handle, uint8_t fsr);
int32_t icm45686_set_accel_odr(Icm45686_Handle_t* handle, uint8_t odr);
int32_t icm45686_set_gyro_odr(Icm45686_Handle_t* handle, uint8_t odr);

int32_t icm45686_set_power_mode(Icm45686_Handle_t* handle,
                                uint8_t accel_mode,
                                uint8_t gyro_mode);

int32_t icm45686_config_int1(Icm45686_Handle_t* handle, uint8_t int_config);
int32_t icm45686_enable_int1(Icm45686_Handle_t* handle, uint8_t int_mask);
int32_t icm45686_disable_int1(Icm45686_Handle_t* handle, uint8_t int_mask);
int32_t icm45686_read_int1_status(Icm45686_Handle_t* handle, uint8_t* status_flags);

int32_t icm45686_self_test(Icm45686_Handle_t* handle,
                           bool* accel_result,
                           bool* gyro_result);

#endif /* ICM45686_H */
