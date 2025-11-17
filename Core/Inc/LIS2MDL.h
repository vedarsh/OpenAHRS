/**
 * @file lis2mdl.h
 * @brief LIS2MDL 3-Axis Magnetometer Driver with SPI Bus Abstraction
 *
 * Hardware: STMicroelectronics LIS2MDL 3-axis digital magnetometer
 * Interface: SPI only (4-wire mode, up to 10 MHz)
 * Platform: Hardware-independent via SPI bus layer
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 17, 2025
 *
 * @note This driver follows NASA JPL and MISRA C coding standards
 * @note All register addresses from LIS2MDL datasheet DS12144 Rev 6
 */

#ifndef LIS2MDL_H
#define LIS2MDL_H

#include "spi_bus.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * DEVICE CONSTANTS
 * ============================================================================ */

#define LIS2MDL_WHO_AM_I_REG            (0x4FU)     /**< WHO_AM_I register address */
#define LIS2MDL_WHO_AM_I_VALUE          (0x40U)     /**< Expected WHO_AM_I response */

/* Timing Constants */
#define LIS2MDL_STARTUP_TIME_MS         (50U)       /**< Device startup time */
#define LIS2MDL_SOFT_RESET_TIME_MS      (5U)        /**< Software reset time */
#define LIS2MDL_MODE_SWITCH_TIME_MS     (10U)       /**< Mode switch time */

/* Physical Constants */
#define LIS2MDL_SENSITIVITY_LSB_GAUSS   (1.5f)      /**< 1.5 mGauss/LSB */
#define LIS2MDL_FS_RANGE_GAUSS          (50.0f)     /**< ±50 gauss range */

/* ============================================================================
 * REGISTER MAP
 * ============================================================================ */

/* Hard-Iron Offset Registers */
#define LIS2MDL_REG_OFFSET_X_L          (0x45U)
#define LIS2MDL_REG_OFFSET_X_H          (0x46U)
#define LIS2MDL_REG_OFFSET_Y_L          (0x47U)
#define LIS2MDL_REG_OFFSET_Y_H          (0x48U)
#define LIS2MDL_REG_OFFSET_Z_L          (0x49U)
#define LIS2MDL_REG_OFFSET_Z_H          (0x4AU)

/* Configuration Registers */
#define LIS2MDL_REG_CFG_A               (0x60U)
#define LIS2MDL_REG_CFG_B               (0x61U)
#define LIS2MDL_REG_CFG_C               (0x62U)
#define LIS2MDL_REG_INT_CTRL            (0x63U)
#define LIS2MDL_REG_INT_SOURCE          (0x64U)
#define LIS2MDL_REG_INT_THS_L           (0x65U)
#define LIS2MDL_REG_INT_THS_H           (0x66U)
#define LIS2MDL_REG_STATUS              (0x67U)

/* Output Data Registers */
#define LIS2MDL_REG_OUTX_L              (0x68U)
#define LIS2MDL_REG_OUTX_H              (0x69U)
#define LIS2MDL_REG_OUTY_L              (0x6AU)
#define LIS2MDL_REG_OUTY_H              (0x6BU)
#define LIS2MDL_REG_OUTZ_L              (0x6CU)
#define LIS2MDL_REG_OUTZ_H              (0x6DU)
#define LIS2MDL_REG_TEMP_OUT_L          (0x6EU)
#define LIS2MDL_REG_TEMP_OUT_H          (0x6FU)

/* ============================================================================
 * CONFIGURATION BIT MASKS
 * ============================================================================ */

/* CFG_REG_A */
#define LIS2MDL_CFG_A_COMP_TEMP_EN      (0x80U)
#define LIS2MDL_CFG_A_REBOOT            (0x40U)
#define LIS2MDL_CFG_A_SOFT_RST          (0x20U)
#define LIS2MDL_CFG_A_LP                (0x10U)
#define LIS2MDL_CFG_A_ODR_MASK          (0x0CU)
#define LIS2MDL_CFG_A_MD_MASK           (0x03U)

/* CFG_REG_B */
#define LIS2MDL_CFG_B_OFF_CANC_ONE_SHOT (0x10U)
#define LIS2MDL_CFG_B_INT_ON_DATAOFF    (0x08U)
#define LIS2MDL_CFG_B_SET_FREQ          (0x04U)
#define LIS2MDL_CFG_B_OFF_CANC          (0x02U)
#define LIS2MDL_CFG_B_LPF               (0x01U)

/* CFG_REG_C */
#define LIS2MDL_CFG_C_INT_ON_PIN        (0x40U)
#define LIS2MDL_CFG_C_I2C_DIS           (0x20U)
#define LIS2MDL_CFG_C_BDU               (0x10U)
#define LIS2MDL_CFG_C_BLE               (0x08U)
#define LIS2MDL_CFG_C_4WSPI             (0x04U)
#define LIS2MDL_CFG_C_SELF_TEST         (0x02U)
#define LIS2MDL_CFG_C_DRDY_ON_PIN       (0x01U)

/* STATUS Register */
#define LIS2MDL_STATUS_ZYXOR            (0x80U)
#define LIS2MDL_STATUS_ZOR              (0x40U)
#define LIS2MDL_STATUS_YOR              (0x20U)
#define LIS2MDL_STATUS_XOR              (0x10U)
#define LIS2MDL_STATUS_ZYXDA            (0x08U)
#define LIS2MDL_STATUS_ZDA              (0x04U)
#define LIS2MDL_STATUS_YDA              (0x02U)
#define LIS2MDL_STATUS_XDA              (0x01U)

/* ============================================================================
 * OPERATING MODES
 * ============================================================================ */

#define LIS2MDL_MODE_CONTINUOUS         (0x00U)
#define LIS2MDL_MODE_SINGLE             (0x01U)
#define LIS2MDL_MODE_IDLE               (0x03U)

/* Output Data Rates */
#define LIS2MDL_ODR_10HZ                (0x00U << 2)
#define LIS2MDL_ODR_20HZ                (0x01U << 2)
#define LIS2MDL_ODR_50HZ                (0x02U << 2)
#define LIS2MDL_ODR_100HZ               (0x03U << 2)

/* ============================================================================
 * ERROR CODES
 * ============================================================================ */

#define LIS2MDL_OK                      (0)
#define LIS2MDL_ERR_BUS                 (-1)
#define LIS2MDL_ERR_TIMEOUT             (-2)
#define LIS2MDL_ERR_INVALID_PARAM       (-3)
#define LIS2MDL_ERR_WHO_AM_I            (-4)
#define LIS2MDL_ERR_NOT_INITIALIZED     (-5)

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

/**
 * @brief Raw sensor data structure
 */
typedef struct {
    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;
    int16_t temp_raw;
} Lis2mdl_RawData_t;

/**
 * @brief Scaled sensor data structure
 */
typedef struct {
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
    float temp_degc;
} Lis2mdl_ScaledData_t;

/**
 * @brief Hard-iron offset structure
 */
typedef struct {
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
} Lis2mdl_Offset_t;

/**
 * @brief Device configuration structure
 */
typedef struct {
    uint8_t odr;
    uint8_t mode;
    bool low_power;
    bool temp_comp;
    bool lpf_enable;
    bool offset_cancel;
    bool bdu_enable;
} Lis2mdl_Config_t;

/**
 * @brief Device handle structure
 */
typedef struct {
    Spi_Device_t spi_device;
    Lis2mdl_Config_t config;
    Lis2mdl_Offset_t offset;
    bool is_initialized;
    float mag_scale_factor;
} Lis2mdl_Handle_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

int32_t lis2mdl_init(Lis2mdl_Handle_t* handle,
                     SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port,
                     uint16_t cs_pin,
                     const Lis2mdl_Config_t* config);

int32_t lis2mdl_read_who_am_i(Lis2mdl_Handle_t* handle, uint8_t* who_am_i);

int32_t lis2mdl_read_raw_data(Lis2mdl_Handle_t* handle,
                              Lis2mdl_RawData_t* data);

int32_t lis2mdl_read_scaled_data(Lis2mdl_Handle_t* handle,
                                 Lis2mdl_ScaledData_t* data);

int32_t lis2mdl_read_status(Lis2mdl_Handle_t* handle, uint8_t* status);

int32_t lis2mdl_soft_reset(Lis2mdl_Handle_t* handle);

int32_t lis2mdl_set_mode(Lis2mdl_Handle_t* handle, uint8_t mode);

int32_t lis2mdl_set_odr(Lis2mdl_Handle_t* handle, uint8_t odr);

int32_t lis2mdl_set_offset(Lis2mdl_Handle_t* handle,
                           const Lis2mdl_Offset_t* offset);

int32_t lis2mdl_get_offset(Lis2mdl_Handle_t* handle,
                           Lis2mdl_Offset_t* offset);

int32_t lis2mdl_self_test(Lis2mdl_Handle_t* handle, bool* result);

int32_t lis2mdl_trigger_measurement(Lis2mdl_Handle_t* handle);

#endif /* LIS2MDL_H */
