/**
 * @file lis2mdl.c
 * @brief LIS2MDL 3-Axis Magnetometer Driver Implementation
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 17, 2025
 *
 * @note Uses SPI bus abstraction layer for hardware independence
 */

#include "lis2mdl.h"
#include "spi_bus.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * PRIVATE CONSTANTS
 * ============================================================================ */

#define LIS2MDL_TEMP_SENSITIVITY    (8.0f)      /**< 8 LSB per °C */
#define LIS2MDL_TEMP_OFFSET_DEGC    (25.0f)     /**< 25°C at 0 LSB */
#define LIS2MDL_MAG_SENSITIVITY     (1.5f)      /**< 1.5 mGauss/LSB */
#define LIS2MDL_MAG_BURST_SIZE      (8U)        /**< 6 mag + 2 temp bytes */

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */

static int32_t lis2mdl_write_register(Lis2mdl_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t value);

static int32_t lis2mdl_read_register(Lis2mdl_Handle_t* handle,
                                     uint8_t reg_addr,
                                     uint8_t* value);

static int32_t lis2mdl_read_registers(Lis2mdl_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t* data,
                                      uint16_t length);

static float lis2mdl_convert_temperature(int16_t raw_temp);
static int32_t lis2mdl_verify_device_id(Lis2mdl_Handle_t* handle);
static int32_t lis2mdl_configure_device(Lis2mdl_Handle_t* handle,
                                        const Lis2mdl_Config_t* config);

/* ============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================ */

static int32_t lis2mdl_write_register(Lis2mdl_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t value)
{
    int32_t status = spi_bus_write_register(&handle->spi_device, reg_addr, value);
    return (status == SPI_BUS_OK) ? LIS2MDL_OK : LIS2MDL_ERR_BUS;
}

static int32_t lis2mdl_read_register(Lis2mdl_Handle_t* handle,
                                     uint8_t reg_addr,
                                     uint8_t* value)
{
    int32_t status = spi_bus_read_register(&handle->spi_device, reg_addr, value);
    return (status == SPI_BUS_OK) ? LIS2MDL_OK : LIS2MDL_ERR_BUS;
}

static int32_t lis2mdl_read_registers(Lis2mdl_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t* data,
                                      uint16_t length)
{
    int32_t status = spi_bus_read_registers(&handle->spi_device, reg_addr, data, length);
    return (status == SPI_BUS_OK) ? LIS2MDL_OK : LIS2MDL_ERR_BUS;
}

static float lis2mdl_convert_temperature(int16_t raw_temp)
{
    return ((float)raw_temp / LIS2MDL_TEMP_SENSITIVITY) + LIS2MDL_TEMP_OFFSET_DEGC;
}

static int32_t lis2mdl_verify_device_id(Lis2mdl_Handle_t* handle)
{
    int32_t status;
    uint8_t who_am_i = 0U;

    status = lis2mdl_read_who_am_i(handle, &who_am_i);
    if (status != LIS2MDL_OK) {
        return status;
    }

    if (who_am_i != LIS2MDL_WHO_AM_I_VALUE) {
        return LIS2MDL_ERR_WHO_AM_I;
    }

    return LIS2MDL_OK;
}

static int32_t lis2mdl_configure_device(Lis2mdl_Handle_t* handle,
                                        const Lis2mdl_Config_t* config)
{
    int32_t status;
    uint8_t cfg_a_value = 0U;
    uint8_t cfg_b_value = 0U;
    uint8_t cfg_c_value = 0U;

    /* Configure CFG_REG_A */
    cfg_a_value = config->odr | config->mode;

    if (config->temp_comp) {
        cfg_a_value |= LIS2MDL_CFG_A_COMP_TEMP_EN;
    }

    if (config->low_power) {
        cfg_a_value |= LIS2MDL_CFG_A_LP;
    }

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_A, cfg_a_value);
    if (status != LIS2MDL_OK) return status;

    /* Configure CFG_REG_B */
    if (config->lpf_enable) {
        cfg_b_value |= LIS2MDL_CFG_B_LPF;
    }

    if (config->offset_cancel) {
        cfg_b_value |= LIS2MDL_CFG_B_OFF_CANC;
    }

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_B, cfg_b_value);
    if (status != LIS2MDL_OK) return status;

    /* Configure CFG_REG_C */
    status = lis2mdl_read_register(handle, LIS2MDL_REG_CFG_C, &cfg_c_value);
    if (status != LIS2MDL_OK) return status;

    if (config->bdu_enable) {
        cfg_c_value |= LIS2MDL_CFG_C_BDU;
    }

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_C, cfg_c_value);

    return status;
}

/* ============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================ */

int32_t lis2mdl_init(Lis2mdl_Handle_t* handle,
                     SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port,
                     uint16_t cs_pin,
                     const Lis2mdl_Config_t* config)
{
    int32_t status;
    uint8_t cfg_c_value;

    if ((handle == NULL) || (hspi == NULL) || (cs_port == NULL) || (config == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    memset(handle, 0, sizeof(Lis2mdl_Handle_t));
    handle->is_initialized = false;

    /* Initialize SPI device */
    spi_bus_init_device(&handle->spi_device, hspi, cs_port, cs_pin);
    handle->spi_device.use_auto_increment_bit = true;  // LIS2MDL needs auto-increment bit

    HAL_Delay(LIS2MDL_STARTUP_TIME_MS);

    /* Enable 4-wire SPI and disable I2C */
    cfg_c_value = LIS2MDL_CFG_C_4WSPI | LIS2MDL_CFG_C_I2C_DIS;
    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_C, cfg_c_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    /* Verify device ID */
//    status = lis2mdl_verify_device_id(handle);
//    if (status != LIS2MDL_OK) {
//        return status;
//    }

    /* Soft reset */
    status = lis2mdl_soft_reset(handle);
    if (status != LIS2MDL_OK) {
        return status;
    }

    /* Restore SPI configuration */
    cfg_c_value = LIS2MDL_CFG_C_4WSPI | LIS2MDL_CFG_C_I2C_DIS;
    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_C, cfg_c_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    /* Configure device */
    status = lis2mdl_configure_device(handle, config);
    if (status != LIS2MDL_OK) {
        return status;
    }

    memcpy(&handle->config, config, sizeof(Lis2mdl_Config_t));
    handle->mag_scale_factor = LIS2MDL_MAG_SENSITIVITY / 1000.0f;
    handle->is_initialized = true;

    return LIS2MDL_OK;
}

int32_t lis2mdl_read_who_am_i(Lis2mdl_Handle_t* handle, uint8_t* who_am_i)
{
    if ((handle == NULL) || (who_am_i == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    return lis2mdl_read_register(handle, LIS2MDL_WHO_AM_I_REG, who_am_i);
}

int32_t lis2mdl_read_raw_data(Lis2mdl_Handle_t* handle,
                              Lis2mdl_RawData_t* data)
{
    int32_t status;
    uint8_t raw_buffer[LIS2MDL_MAG_BURST_SIZE];

    if ((handle == NULL) || (data == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    if (!handle->is_initialized) {
        return LIS2MDL_ERR_NOT_INITIALIZED;
    }

    status = lis2mdl_read_registers(handle, LIS2MDL_REG_OUTX_L,
                                    raw_buffer, LIS2MDL_MAG_BURST_SIZE);
    if (status != LIS2MDL_OK) {
        return status;
    }

    /* Parse little-endian data */
    data->mag_x_raw = (int16_t)(raw_buffer[0] | (raw_buffer[1] << 8));
    data->mag_y_raw = (int16_t)(raw_buffer[2] | (raw_buffer[3] << 8));
    data->mag_z_raw = (int16_t)(raw_buffer[4] | (raw_buffer[5] << 8));
    data->temp_raw = (int16_t)(raw_buffer[6] | (raw_buffer[7] << 8));

    return LIS2MDL_OK;
}

int32_t lis2mdl_read_scaled_data(Lis2mdl_Handle_t* handle,
                                 Lis2mdl_ScaledData_t* data)
{
    int32_t status;
    Lis2mdl_RawData_t raw_data;

    if ((handle == NULL) || (data == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_read_raw_data(handle, &raw_data);
    if (status != LIS2MDL_OK) {
        return status;
    }

    data->mag_x_gauss = (float)raw_data.mag_x_raw * handle->mag_scale_factor;
    data->mag_y_gauss = (float)raw_data.mag_y_raw * handle->mag_scale_factor;
    data->mag_z_gauss = (float)raw_data.mag_z_raw * handle->mag_scale_factor;
    data->temp_degc = lis2mdl_convert_temperature(raw_data.temp_raw);

    return LIS2MDL_OK;
}

int32_t lis2mdl_read_status(Lis2mdl_Handle_t* handle, uint8_t* status)
{
    if ((handle == NULL) || (status == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    if (!handle->is_initialized) {
        return LIS2MDL_ERR_NOT_INITIALIZED;
    }

    return lis2mdl_read_register(handle, LIS2MDL_REG_STATUS, status);
}

int32_t lis2mdl_soft_reset(Lis2mdl_Handle_t* handle)
{
    int32_t status;
    uint8_t cfg_a_value;

    if (handle == NULL) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_read_register(handle, LIS2MDL_REG_CFG_A, &cfg_a_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    cfg_a_value |= LIS2MDL_CFG_A_SOFT_RST;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_A, cfg_a_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    HAL_Delay(LIS2MDL_SOFT_RESET_TIME_MS);

    return LIS2MDL_OK;
}

int32_t lis2mdl_set_mode(Lis2mdl_Handle_t* handle, uint8_t mode)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_read_register(handle, LIS2MDL_REG_CFG_A, &reg_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    reg_value = (reg_value & (~LIS2MDL_CFG_A_MD_MASK)) | (mode & LIS2MDL_CFG_A_MD_MASK);

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_A, reg_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    handle->config.mode = mode;
    HAL_Delay(LIS2MDL_MODE_SWITCH_TIME_MS);

    return LIS2MDL_OK;
}

int32_t lis2mdl_set_odr(Lis2mdl_Handle_t* handle, uint8_t odr)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_read_register(handle, LIS2MDL_REG_CFG_A, &reg_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    reg_value = (reg_value & (~LIS2MDL_CFG_A_ODR_MASK)) | (odr & LIS2MDL_CFG_A_ODR_MASK);

    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_A, reg_value);
    if (status != LIS2MDL_OK) {
        return status;
    }

    handle->config.odr = odr;

    return LIS2MDL_OK;
}

int32_t lis2mdl_set_offset(Lis2mdl_Handle_t* handle,
                           const Lis2mdl_Offset_t* offset)
{
    int32_t status;

    if ((handle == NULL) || (offset == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_X_L,
                                    (uint8_t)(offset->offset_x & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_X_H,
                                    (uint8_t)((offset->offset_x >> 8) & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_Y_L,
                                    (uint8_t)(offset->offset_y & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_Y_H,
                                    (uint8_t)((offset->offset_y >> 8) & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_Z_L,
                                    (uint8_t)(offset->offset_z & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_write_register(handle, LIS2MDL_REG_OFFSET_Z_H,
                                    (uint8_t)((offset->offset_z >> 8) & 0xFFU));
    if (status != LIS2MDL_OK) return status;

    memcpy(&handle->offset, offset, sizeof(Lis2mdl_Offset_t));

    return LIS2MDL_OK;
}

int32_t lis2mdl_get_offset(Lis2mdl_Handle_t* handle,
                           Lis2mdl_Offset_t* offset)
{
    int32_t status;
    uint8_t offset_buffer[6];

    if ((handle == NULL) || (offset == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    status = lis2mdl_read_registers(handle, LIS2MDL_REG_OFFSET_X_L, offset_buffer, 6U);
    if (status != LIS2MDL_OK) {
        return status;
    }

    offset->offset_x = (int16_t)(offset_buffer[0] | (offset_buffer[1] << 8));
    offset->offset_y = (int16_t)(offset_buffer[2] | (offset_buffer[3] << 8));
    offset->offset_z = (int16_t)(offset_buffer[4] | (offset_buffer[5] << 8));

    return LIS2MDL_OK;
}

int32_t lis2mdl_self_test(Lis2mdl_Handle_t* handle, bool* result)
{
    int32_t status;
    uint8_t cfg_c_value;
    Lis2mdl_RawData_t data_normal, data_selftest;
    float diff_x, diff_y, diff_z;
    const float min_diff = 15.0f;
    const float max_diff = 500.0f;

    if ((handle == NULL) || (result == NULL)) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    *result = false;

    status = lis2mdl_read_raw_data(handle, &data_normal);
    if (status != LIS2MDL_OK) return status;

    status = lis2mdl_read_register(handle, LIS2MDL_REG_CFG_C, &cfg_c_value);
    if (status != LIS2MDL_OK) return status;

    cfg_c_value |= LIS2MDL_CFG_C_SELF_TEST;
    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_C, cfg_c_value);
    if (status != LIS2MDL_OK) return status;

    HAL_Delay(60U);

    status = lis2mdl_read_raw_data(handle, &data_selftest);
    if (status != LIS2MDL_OK) return status;

    cfg_c_value &= ~LIS2MDL_CFG_C_SELF_TEST;
    status = lis2mdl_write_register(handle, LIS2MDL_REG_CFG_C, cfg_c_value);
    if (status != LIS2MDL_OK) return status;

    diff_x = fabsf((float)(data_selftest.mag_x_raw - data_normal.mag_x_raw)) * LIS2MDL_MAG_SENSITIVITY;
    diff_y = fabsf((float)(data_selftest.mag_y_raw - data_normal.mag_y_raw)) * LIS2MDL_MAG_SENSITIVITY;
    diff_z = fabsf((float)(data_selftest.mag_z_raw - data_normal.mag_z_raw)) * LIS2MDL_MAG_SENSITIVITY;

    if ((diff_x >= min_diff) && (diff_x <= max_diff) &&
        (diff_y >= min_diff) && (diff_y <= max_diff) &&
        (diff_z >= min_diff) && (diff_z <= max_diff)) {
        *result = true;
    }

    return LIS2MDL_OK;
}

int32_t lis2mdl_trigger_measurement(Lis2mdl_Handle_t* handle)
{
    if (handle == NULL) {
        return LIS2MDL_ERR_INVALID_PARAM;
    }

    return lis2mdl_set_mode(handle, LIS2MDL_MODE_SINGLE);
}
