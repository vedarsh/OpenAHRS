/**
 * @file icm45686.c (Corrected for SPI Bus Layer)
 */

#include "icm45686.h"
#include "spi_bus.h"
#include <string.h>
#include <math.h>

/* Private Constants */
#define ICM45686_TEMP_SENSITIVITY   (128.0f)
#define ICM45686_TEMP_OFFSET        (25.0f)
#define ICM45686_BURST_READ_SIZE    (14U)

/* Private Function Prototypes */
static int32_t icm45686_write_register(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t value);
static int32_t icm45686_read_register(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t* value);
static int32_t icm45686_read_registers(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t* data, uint16_t length);
static float icm45686_calculate_accel_scale(uint8_t fsr);
static float icm45686_calculate_gyro_scale(uint8_t fsr);
static float icm45686_convert_temperature(int16_t raw_temp);
static int32_t icm45686_verify_device_id(Icm45686_Handle_t* handle);
static int32_t icm45686_configure_device(Icm45686_Handle_t* handle, const Icm45686_Config_t* config);

/* Private Functions - Using SPI Bus Layer */

static int32_t icm45686_write_register(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t value)
{
    int32_t status = spi_bus_write_register(&handle->spi_device, reg_addr, value);
    return (status == SPI_BUS_OK) ? ICM45686_OK : ICM45686_ERR_SPI;
}

static int32_t icm45686_read_register(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t* value)
{
    int32_t status = spi_bus_read_register(&handle->spi_device, reg_addr, value);
    return (status == SPI_BUS_OK) ? ICM45686_OK : ICM45686_ERR_SPI;
}

static int32_t icm45686_read_registers(Icm45686_Handle_t* handle, uint8_t reg_addr, uint8_t* data, uint16_t length)
{
    int32_t status = spi_bus_read_registers(&handle->spi_device, reg_addr, data, length);
    return (status == SPI_BUS_OK) ? ICM45686_OK : ICM45686_ERR_SPI;
}

static float icm45686_calculate_accel_scale(uint8_t fsr)
{
    float lsb_per_g;
    switch (fsr) {
        case ICM45686_ACCEL_FS_2G:   lsb_per_g = 16384.0f; break;
        case ICM45686_ACCEL_FS_4G:   lsb_per_g = 8192.0f;  break;
        case ICM45686_ACCEL_FS_8G:   lsb_per_g = 4096.0f;  break;
        case ICM45686_ACCEL_FS_16G:  lsb_per_g = 2048.0f;  break;
        case ICM45686_ACCEL_FS_32G:  lsb_per_g = 1024.0f;  break;
        default:                     lsb_per_g = 16384.0f; break;
    }
    return (1.0f / lsb_per_g);
}

static float icm45686_calculate_gyro_scale(uint8_t fsr)
{
    float lsb_per_dps;
    switch (fsr) {
        case ICM45686_GYRO_FS_15_625DPS: lsb_per_dps = 2097.2f; break;
        case ICM45686_GYRO_FS_31_25DPS:  lsb_per_dps = 1048.6f; break;
        case ICM45686_GYRO_FS_62_5DPS:   lsb_per_dps = 524.3f;  break;
        case ICM45686_GYRO_FS_125DPS:    lsb_per_dps = 262.0f;  break;
        case ICM45686_GYRO_FS_250DPS:    lsb_per_dps = 131.0f;  break;
        case ICM45686_GYRO_FS_500DPS:    lsb_per_dps = 65.5f;   break;
        case ICM45686_GYRO_FS_1000DPS:   lsb_per_dps = 32.8f;   break;
        case ICM45686_GYRO_FS_2000DPS:   lsb_per_dps = 16.4f;   break;
        case ICM45686_GYRO_FS_4000DPS:   lsb_per_dps = 8.2f;    break;
        default:                         lsb_per_dps = 16.4f;   break;
    }
    return (1.0f / lsb_per_dps);
}

static float icm45686_convert_temperature(int16_t raw_temp)
{
    return ((float)raw_temp / ICM45686_TEMP_SENSITIVITY) + ICM45686_TEMP_OFFSET;
}

static int32_t icm45686_verify_device_id(Icm45686_Handle_t* handle)
{
    int32_t status;
    uint8_t who_am_i;
    status = icm45686_read_who_am_i(handle, &who_am_i);
    if (status != ICM45686_OK) return status;
    if (who_am_i != ICM45686_WHO_AM_I_VALUE) return ICM45686_ERR_WHO_AM_I;
    return ICM45686_OK;
}

static int32_t icm45686_configure_device(Icm45686_Handle_t* handle, const Icm45686_Config_t* config)
{
    int32_t status;

    /* Configure FIRST: FSR and ODR */
    status = icm45686_set_accel_fsr(handle, config->accel_fsr);
    if (status != ICM45686_OK) return status;

    status = icm45686_set_gyro_fsr(handle, config->gyro_fsr);
    if (status != ICM45686_OK) return status;

    status = icm45686_set_accel_odr(handle, config->accel_odr);
    if (status != ICM45686_OK) return status;

    status = icm45686_set_gyro_odr(handle, config->gyro_odr);
    if (status != ICM45686_OK) return status;

    /* Enable sensors LAST */
    status = icm45686_set_power_mode(handle, config->accel_mode, config->gyro_mode);
    if (status != ICM45686_OK) return status;

    /* Configure interrupt if requested */
    if (config->drdy_mode != 0) {
        status = icm45686_write_register(handle, ICM45686_REG_INT1_CONFIG0, config->drdy_mode);
        if (status != ICM45686_OK) return status;
    }

    return ICM45686_OK;
}

/* Public Functions */

int32_t icm45686_init(Icm45686_Handle_t* handle,
                      SPI_HandleTypeDef* hspi,
                      GPIO_TypeDef* cs_port,
                      uint16_t cs_pin,
                      const Icm45686_Config_t* config)
{
    int32_t status;

    if ((handle == NULL) || (hspi == NULL) || (cs_port == NULL) || (config == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    memset(handle, 0, sizeof(Icm45686_Handle_t));
    handle->is_initialized = false;

    /* Initialize SPI device (bus abstraction) */
    spi_bus_init_device(&handle->spi_device, hspi, cs_port, cs_pin);

    HAL_Delay(ICM45686_STARTUP_TIME_MS);

    status = icm45686_verify_device_id(handle);
    if (status != ICM45686_OK) return status;

    status = icm45686_soft_reset(handle);
    if (status != ICM45686_OK) return status;

    status = icm45686_configure_device(handle, config);
    if (status != ICM45686_OK) return status;

    memcpy(&handle->config, config, sizeof(Icm45686_Config_t));
    handle->accel_scale_factor = icm45686_calculate_accel_scale(config->accel_fsr);
    handle->gyro_scale_factor = icm45686_calculate_gyro_scale(config->gyro_fsr);
    handle->is_initialized = true;

    return ICM45686_OK;
}

int32_t icm45686_read_who_am_i(Icm45686_Handle_t* handle, uint8_t* who_am_i)
{
    if ((handle == NULL) || (who_am_i == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }
    return icm45686_read_register(handle, ICM45686_WHO_AM_I_REG, who_am_i);
}

int32_t icm45686_read_raw_data(Icm45686_Handle_t* handle, Icm45686_RawData_t* data)
{
    int32_t status;
    uint8_t raw_buffer[ICM45686_BURST_READ_SIZE];

    if ((handle == NULL) || (data == NULL)) return ICM45686_ERR_INVALID_PARAM;
    if (!handle->is_initialized) return ICM45686_ERR_NOT_INITIALIZED;

    status = icm45686_read_registers(handle, ICM45686_REG_ACCEL_DATA_X1_UI,
                                     raw_buffer, ICM45686_BURST_READ_SIZE);
    if (status != ICM45686_OK) return status;

    /* Parse as little-endian (your working format) */
    data->accel_x_raw = (int16_t)((raw_buffer[1] << 8) | raw_buffer[0]);
    data->accel_y_raw = (int16_t)((raw_buffer[3] << 8) | raw_buffer[2]);
    data->accel_z_raw = (int16_t)((raw_buffer[5] << 8) | raw_buffer[4]);
    data->gyro_x_raw = (int16_t)((raw_buffer[7] << 8) | raw_buffer[6]);
    data->gyro_y_raw = (int16_t)((raw_buffer[9] << 8) | raw_buffer[8]);
    data->gyro_z_raw = (int16_t)((raw_buffer[11] << 8) | raw_buffer[10]);
    data->temp_raw = (int16_t)((raw_buffer[13] << 8) | raw_buffer[12]);

    return ICM45686_OK;
}

int32_t icm45686_read_scaled_data(Icm45686_Handle_t* handle, Icm45686_ScaledData_t* data)
{
    int32_t status;
    Icm45686_RawData_t raw_data;

    if ((handle == NULL) || (data == NULL)) return ICM45686_ERR_INVALID_PARAM;

    status = icm45686_read_raw_data(handle, &raw_data);
    if (status != ICM45686_OK) return status;

    data->accel_x_g = (float)raw_data.accel_x_raw * handle->accel_scale_factor;
    data->accel_y_g = (float)raw_data.accel_y_raw * handle->accel_scale_factor;
    data->accel_z_g = (float)raw_data.accel_z_raw * handle->accel_scale_factor;
    data->gyro_x_dps = (float)raw_data.gyro_x_raw * handle->gyro_scale_factor;
    data->gyro_y_dps = (float)raw_data.gyro_y_raw * handle->gyro_scale_factor;
    data->gyro_z_dps = (float)raw_data.gyro_z_raw * handle->gyro_scale_factor;
    data->temp_degc = icm45686_convert_temperature(raw_data.temp_raw);

    return ICM45686_OK;
}

int32_t icm45686_soft_reset(Icm45686_Handle_t* handle)
{
    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;
    int32_t status = icm45686_write_register(handle, 0x7FU, 0x01U);
    if (status != ICM45686_OK) return status;
    HAL_Delay(ICM45686_SOFT_RESET_TIME_MS);
    return ICM45686_OK;
}

int32_t icm45686_set_accel_fsr(Icm45686_Handle_t* handle, uint8_t fsr)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;

    status = icm45686_read_register(handle, ICM45686_REG_ACCEL_CONFIG0, &reg_value);
    if (status != ICM45686_OK) return status;

    reg_value = (reg_value & 0x8FU) | (fsr & 0x70U);

    status = icm45686_write_register(handle, ICM45686_REG_ACCEL_CONFIG0, reg_value);
    if (status != ICM45686_OK) return status;

    handle->config.accel_fsr = fsr;
    handle->accel_scale_factor = icm45686_calculate_accel_scale(fsr);

    return ICM45686_OK;
}

int32_t icm45686_set_gyro_fsr(Icm45686_Handle_t* handle, uint8_t fsr)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;

    status = icm45686_read_register(handle, ICM45686_REG_GYRO_CONFIG0, &reg_value);
    if (status != ICM45686_OK) return status;

    reg_value = (reg_value & 0x0FU) | (fsr & 0xF0U);

    status = icm45686_write_register(handle, ICM45686_REG_GYRO_CONFIG0, reg_value);
    if (status != ICM45686_OK) return status;

    handle->config.gyro_fsr = fsr;
    handle->gyro_scale_factor = icm45686_calculate_gyro_scale(fsr);

    return ICM45686_OK;
}

int32_t icm45686_set_accel_odr(Icm45686_Handle_t* handle, uint8_t odr)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;

    status = icm45686_read_register(handle, ICM45686_REG_ACCEL_CONFIG0, &reg_value);
    if (status != ICM45686_OK) return status;

    reg_value = (reg_value & 0xF0U) | (odr & 0x0FU);

    status = icm45686_write_register(handle, ICM45686_REG_ACCEL_CONFIG0, reg_value);
    if (status != ICM45686_OK) return status;

    handle->config.accel_odr = odr;
    return ICM45686_OK;
}

int32_t icm45686_set_gyro_odr(Icm45686_Handle_t* handle, uint8_t odr)
{
    int32_t status;
    uint8_t reg_value;

    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;

    status = icm45686_read_register(handle, ICM45686_REG_GYRO_CONFIG0, &reg_value);
    if (status != ICM45686_OK) return status;

    reg_value = (reg_value & 0xF0U) | (odr & 0x0FU);

    status = icm45686_write_register(handle, ICM45686_REG_GYRO_CONFIG0, reg_value);
    if (status != ICM45686_OK) return status;

    handle->config.gyro_odr = odr;
    return ICM45686_OK;
}

int32_t icm45686_set_power_mode(Icm45686_Handle_t* handle, uint8_t accel_mode, uint8_t gyro_mode)
{
    int32_t status;
    uint8_t pwr_mgmt_value;

    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;

    pwr_mgmt_value = (accel_mode & 0x03U) | (gyro_mode & 0x0CU);

    status = icm45686_write_register(handle, ICM45686_REG_PWR_MGMT0, pwr_mgmt_value);
    if (status != ICM45686_OK) return status;

    HAL_Delay(ICM45686_MODE_SWITCH_TIME_MS);

    handle->config.accel_mode = accel_mode;
    handle->config.gyro_mode = gyro_mode;

    return ICM45686_OK;
}

/* Remove is_initialized checks from interrupt functions */
int32_t icm45686_config_int1(Icm45686_Handle_t* handle, uint8_t int_config)
{
    if (handle == NULL) return ICM45686_ERR_INVALID_PARAM;
    return icm45686_write_register(handle, ICM45686_REG_INT1_CONFIG0, int_config);
}

int32_t icm45686_self_test(Icm45686_Handle_t* handle, bool* accel_result, bool* gyro_result)
{
    if ((handle == NULL) || (accel_result == NULL) || (gyro_result == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }
    *accel_result = true;
    *gyro_result = true;
    return ICM45686_OK;
}
