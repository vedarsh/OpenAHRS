#ifndef LIS2MDL_H
#define LIS2MDL_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "spi_types.h"

/* =========================================================================
 *  Device identification
 * ========================================================================= */
#define LIS2MDL_CHIP_ID            0x40U

/* =========================================================================
 *  Register addresses
 * ========================================================================= */
#define LIS2MDL_REG_OFFSET_X_L     0x45U
#define LIS2MDL_REG_OFFSET_X_H     0x46U
#define LIS2MDL_REG_OFFSET_Y_L     0x47U
#define LIS2MDL_REG_OFFSET_Y_H     0x48U
#define LIS2MDL_REG_OFFSET_Z_L     0x49U
#define LIS2MDL_REG_OFFSET_Z_H     0x4AU

#define LIS2MDL_REG_WHO_AM_I       0x4FU

#define LIS2MDL_REG_CFG_A          0x60U
#define LIS2MDL_REG_CFG_B          0x61U
#define LIS2MDL_REG_CFG_C          0x62U

#define LIS2MDL_REG_STATUS         0x67U

#define LIS2MDL_REG_OUTX_L         0x68U
#define LIS2MDL_REG_OUTX_H         0x69U
#define LIS2MDL_REG_OUTY_L         0x6AU
#define LIS2MDL_REG_OUTY_H         0x6BU
#define LIS2MDL_REG_OUTZ_L         0x6CU
#define LIS2MDL_REG_OUTZ_H         0x6DU

#define LIS2MDL_REG_TEMP_L         0x6EU
#define LIS2MDL_REG_TEMP_H         0x6FU

/* =========================================================================
 *  CFG_REG_A (0x60) bit definitions
 * =========================================================================
 *  bit7     COMP_TEMP_EN
 *  bit6     REBOOT
 *  bit5     SOFT_RST
 *  bit4     LP
 *  bit3:2   ODR[1:0]
 *  bit1:0   MODE[1:0]
 */
#define LIS2MDL_CFG_A_COMP_TEMP_EN   (1U << 7)
#define LIS2MDL_CFG_A_REBOOT         (1U << 6)
#define LIS2MDL_CFG_A_SOFT_RST       (1U << 5)
#define LIS2MDL_CFG_A_LP             (1U << 4)

/* ODR field */
#define LIS2MDL_CFG_A_ODR_SHIFT      2
#define LIS2MDL_CFG_A_ODR_MASK       (0x3U << LIS2MDL_CFG_A_ODR_SHIFT)

/* Mode field */
#define LIS2MDL_CFG_A_MODE_SHIFT     0
#define LIS2MDL_CFG_A_MODE_MASK      (0x3U << LIS2MDL_CFG_A_MODE_SHIFT)

/* ODR values (unshifted) */
typedef enum {
    LIS2MDL_ODR_10HZ  = 0,
    LIS2MDL_ODR_20HZ  = 1,
    LIS2MDL_ODR_50HZ  = 2,
    LIS2MDL_ODR_100HZ = 3
} lis2mdl_odr_t;

/* Operating modes */
typedef enum {
    LIS2MDL_MODE_CONTINUOUS = 0,
    LIS2MDL_MODE_SINGLE     = 1,
    LIS2MDL_MODE_IDLE       = 3
} lis2mdl_mode_t;

/* =========================================================================
 *  CFG_REG_B (0x61) bit definitions
 * ========================================================================= */
#define LIS2MDL_CFG_B_LPF_EN         (1U << 0)
#define LIS2MDL_CFG_B_OFF_CANC_EN    (1U << 1)

/* =========================================================================
 *  CFG_REG_C (0x62) bit definitions
 * ========================================================================= */
#define LIS2MDL_CFG_C_DRDY_ON_PIN    (1U << 3)
#define LIS2MDL_CFG_C_BDU            (1U << 4)
#define LIS2MDL_CFG_C_I2C_DISABLE    (1U << 5)
#define LIS2MDL_CFG_C_SPI_4WIRE      (1 << 2)

#define LIS2MDL_CFG_B_SET_RST_SHIFT   4
#define LIS2MDL_CFG_B_SET_RST_MASK    (0x3 << LIS2MDL_CFG_B_SET_RST_SHIFT)

#define LIS2MDL_SET_RST_ODR_DIV_63        0
#define LIS2MDL_SET_RST_EVERY_ODR         1
#define LIS2MDL_SET_RST_POWER_ON_ONLY    2


/* =========================================================================
 *  STATUS_REG (0x67)
 * ========================================================================= */
#define LIS2MDL_STATUS_ZYXDA         (1U << 3)


/* =========================================================================
 *  Data containers
 * ========================================================================= */
typedef struct {
    float x_ut;
    float y_ut;
    float z_ut;

} lis2mdl_data_t;

/* =========================================================================
 *  Utility macros
 * ========================================================================= */
#define LIS2MDL_WRITE_FIELD(reg, mask, shift, val) \
    do { (reg) = ((reg) & ~(mask)) | (((val) << (shift)) & (mask)); } while (0)


/* ======================= Core control ======================= */

int32_t lis2mdl_read_chip_id(const spi_bus_t *bus);
int32_t lis2mdl_soft_reset(const spi_bus_t *bus);

int32_t lis2mdl_set_odr(const spi_bus_t *bus, lis2mdl_odr_t odr);
int32_t lis2mdl_set_mode(const spi_bus_t *bus, lis2mdl_mode_t mode);
int32_t lis2mdl_enable_compensation(const spi_bus_t *bus, bool enable);

int32_t lis2mdl_config_filters(const spi_bus_t *bus,
                               bool lpf,
                               bool offset_cancel);

int32_t lis2mdl_config_interface(const spi_bus_t *bus,
                                 bool bdu,
                                 bool i2c_disable);

/* ======================= Data read ======================= */

int32_t lis2mdl_read_raw(const spi_bus_t *bus,
                         int16_t *x,
                         int16_t *y,
                         int16_t *z);

int32_t lis2mdl_read_mag(const spi_bus_t *bus,
                         lis2mdl_data_t *data);

int32_t lis2mdl_read_temperature(const spi_bus_t *bus,
                                 float *temp_c);

int32_t lis2mdl_set_spi_mode(const spi_bus_t *bus, bool spi_4wire);
bool lis2mdl_read_sensor_ready(const spi_bus_t *bus);

#endif /* LIS2MDL_H */
