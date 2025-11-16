/*
 * LIS2MDL.h
 *
 *  Created on: Nov 16, 2025
 *      Author: vedarshreddymuniratnam
 */

#ifndef INC_LIS2MDL_H_
#define INC_LIS2MDL_H_

#include "stm32f4xx.h"

#include <stdint.h>
#include <stdbool.h>

#define MAX_SENSOR_SPEED_SPI  10U; //MAX speed is 10MHz need to tune it down for imu.

/** @defgroup Magnetometer_Register_Addresses Register Address Definitions */
/** @{ */

/* Hard-Iron Offset Registers */
#define MAG_REG_OFFSET_X_L              (0x45U)  /**< X-axis offset low byte */
#define MAG_REG_OFFSET_X_H              (0x46U)  /**< X-axis offset high byte */
#define MAG_REG_OFFSET_Y_L              (0x47U)  /**< Y-axis offset low byte */
#define MAG_REG_OFFSET_Y_H              (0x48U)  /**< Y-axis offset high byte */
#define MAG_REG_OFFSET_Z_L              (0x49U)  /**< Z-axis offset low byte */
#define MAG_REG_OFFSET_Z_H              (0x4AU)  /**< Z-axis offset high byte */

/* Device Identification */
#define MAG_REG_WHO_AM_I                (0x4FU)  /**< WHO_AM_I register address */
#define MAG_WHO_AM_I_VALUE              (0x40U)  /**< Expected WHO_AM_I response */

/* Configuration Registers */
#define MAG_REG_CFG_A                   (0x60U)  /**< Configuration register A */
#define MAG_REG_CFG_B                   (0x61U)  /**< Configuration register B */
#define MAG_REG_CFG_C                   (0x62U)  /**< Configuration register C */

/* Interrupt Registers */
#define MAG_REG_INT_CTRL                (0x63U)  /**< Interrupt control register */
#define MAG_REG_INT_SOURCE              (0x64U)  /**< Interrupt source register (read-only) */
#define MAG_REG_INT_THS_L               (0x65U)  /**< Interrupt threshold low byte */
#define MAG_REG_INT_THS_H               (0x66U)  /**< Interrupt threshold high byte */

/* Status Register */
#define MAG_REG_STATUS                  (0x67U)  /**< Status register (read-only) */

/* Magnetic Field Output Registers */
#define MAG_REG_OUTX_L                  (0x68U)  /**< X-axis output low byte */
#define MAG_REG_OUTX_H                  (0x69U)  /**< X-axis output high byte */
#define MAG_REG_OUTY_L                  (0x6AU)  /**< Y-axis output low byte */
#define MAG_REG_OUTY_H                  (0x6BU)  /**< Y-axis output high byte */
#define MAG_REG_OUTZ_L                  (0x6CU)  /**< Z-axis output low byte */
#define MAG_REG_OUTZ_H                  (0x6DU)  /**< Z-axis output high byte */

/* Temperature Sensor Registers */
#define MAG_REG_TEMP_OUT_L              (0x6EU)  /**< Temperature output low byte */
#define MAG_REG_TEMP_OUT_H              (0x6FU)  /**< Temperature output high byte */

/** @} */

/* Configuration Register A Bit Masks */
#define MAG_CFG_A_COMP_TEMP_EN          (0x80U)  /**< Temperature compensation enable */
#define MAG_CFG_A_REBOOT                (0x40U)  /**< Reboot memory content */
#define MAG_CFG_A_SOFT_RST              (0x20U)  /**< Soft reset */
#define MAG_CFG_A_LP                    (0x10U)  /**< Low-power mode enable */
#define MAG_CFG_A_ODR_MASK              (0x0CU)  /**< Output data rate mask */
#define MAG_CFG_A_MD_MASK               (0x03U)  /**< Operating mode mask */

/* Configuration Register B Bit Masks */
#define MAG_CFG_B_OFF_CANC_ONE_SHOT     (0x10U)  /**< Offset cancellation in single mode */
#define MAG_CFG_B_INT_ON_DATAOFF        (0x08U)  /**< Interrupt on data off */
#define MAG_CFG_B_SET_FREQ              (0x04U)  /**< Set pulse frequency */
#define MAG_CFG_B_OFF_CANC              (0x02U)  /**< Offset cancellation */
#define MAG_CFG_B_LPF                   (0x01U)  /**< Low-pass filter enable */

/* Configuration Register C Bit Masks */
#define MAG_CFG_C_INT_ON_PIN            (0x40U)  /**< Interrupt on INT pin */
#define MAG_CFG_C_I2C_DIS               (0x20U)  /**< I2C interface disable */
#define MAG_CFG_C_BDU                   (0x10U)  /**< Block data update */
#define MAG_CFG_C_BLE                   (0x08U)  /**< Big/little endian selection */
#define MAG_CFG_C_4WSPI                 (0x04U)  /**< 4-wire SPI mode */
#define MAG_CFG_C_SELF_TEST             (0x02U)  /**< Self-test enable */
#define MAG_CFG_C_DRDY_ON_PIN           (0x01U)  /**< Data ready on INT pin */

/* Interrupt Control Register Bit Masks */
#define MAG_INT_CTRL_XIEN               (0x80U)  /**< X-axis interrupt enable */
#define MAG_INT_CTRL_YIEN               (0x40U)  /**< Y-axis interrupt enable */
#define MAG_INT_CTRL_ZIEN               (0x20U)  /**< Z-axis interrupt enable */
#define MAG_INT_CTRL_IEA                (0x04U)  /**< Interrupt active high/low */
#define MAG_INT_CTRL_IEL                (0x02U)  /**< Interrupt latched/pulsed */
#define MAG_INT_CTRL_IEN                (0x01U)  /**< Interrupt enable */

/* Status Register Bit Masks */
#define MAG_STATUS_ZYXOR                (0x80U)  /**< X, Y, Z-axis data overrun */
#define MAG_STATUS_ZOR                  (0x40U)  /**< Z-axis data overrun */
#define MAG_STATUS_YOR                  (0x20U)  /**< Y-axis data overrun */
#define MAG_STATUS_XOR                  (0x10U)  /**< X-axis data overrun */
#define MAG_STATUS_ZYXDA                (0x08U)  /**< X, Y, Z-axis data available */
#define MAG_STATUS_ZDA                  (0x04U)  /**< Z-axis data available */
#define MAG_STATUS_YDA                  (0x02U)  /**< Y-axis data available */
#define MAG_STATUS_XDA                  (0x01U)  /**< X-axis data available */

/* Operating Mode Values */
#define MAG_MODE_CONTINUOUS             (0x00U)  /**< Continuous conversion mode */
#define MAG_MODE_SINGLE                 (0x01U)  /**< Single conversion mode */
#define MAG_MODE_IDLE                   (0x03U)  /**< Idle mode (power down) */

/* Output Data Rate Values (CFG_REG_A bits [3:2]) */
#define MAG_ODR_10HZ                    (0x00U << 2)  /**< 10 Hz */
#define MAG_ODR_20HZ                    (0x01U << 2)  /**< 20 Hz */
#define MAG_ODR_50HZ                    (0x02U << 2)  /**< 50 Hz */
#define MAG_ODR_100HZ                   (0x03U << 2)  /**< 100 Hz */

/* Reset Values */
#define MAG_CFG_A_RESET                 (0x03U)  /**< CFG_REG_A reset value */
#define MAG_CFG_B_RESET                 (0x00U)  /**< CFG_REG_B reset value */
#define MAG_CFG_C_RESET                 (0x00U)  /**< CFG_REG_C reset value */
#define MAG_INT_CTRL_RESET              (0xE0U)  /**< INT_CTRL_REG reset value */


typedef struct {
	uint8_t
};

#endif /* INC_LIS2MDL_H_ */
