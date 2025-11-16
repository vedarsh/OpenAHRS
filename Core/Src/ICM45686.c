/**
 * @file icm45686.c
 * @brief ICM-45686 6-Axis IMU SPI Driver Implementation with DMA Support
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 14, 2025
 *
 */

#include "icm45686.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 * PRIVATE CONSTANTS
 * ============================================================================ */

/* Temperature Sensor Conversion Constants (from datasheet section 4.12) */
#define ICM45686_TEMP_SENSITIVITY   (128.0f)    /**< LSB per degree C */
#define ICM45686_TEMP_OFFSET        (25.0f)     /**< Temperature offset in C */

/* Gravity constant for acceleration conversion */
#define GRAVITY_MPS2                (9.80665f)  /**< Standard gravity m/s² */

/* DMA Transfer Sizes */
#define ICM45686_BURST_READ_SIZE    (14U)       /**< All sensor data bytes */
#define ICM45686_REG_ADDR_SIZE      (1U)        /**< Register address size */

/* Register Access Masks */
#define ICM45686_REG_BANK_SEL_MASK  (0x0FU)     /**< Register bank mask */

/* Maximum retry attempts for operations */
#define ICM45686_MAX_RETRIES        (3U)        /**< Max SPI retry count */

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */

/* DMA transfer buffers */
static uint8_t spi_tx_buffer[ICM45686_BURST_READ_SIZE + ICM45686_REG_ADDR_SIZE];
static uint8_t spi_rx_buffer[ICM45686_BURST_READ_SIZE + ICM45686_REG_ADDR_SIZE];

/* DMA transfer complete flag (volatile for ISR access) */
static volatile bool dma_transfer_complete = false;

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */

static int32_t icm45686_write_register(Icm45686_Handle_t* handle,
                                       uint8_t reg_addr,
                                       uint8_t value);

static int32_t icm45686_read_register(Icm45686_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t* value);

static int32_t icm45686_read_registers_dma(Icm45686_Handle_t* handle,
                                           uint8_t reg_addr,
                                           uint8_t* data,
                                           uint16_t length);

static void icm45686_cs_low(Icm45686_Handle_t* handle);
static void icm45686_cs_high(Icm45686_Handle_t* handle);

static float icm45686_calculate_accel_scale(uint8_t fsr);
static float icm45686_calculate_gyro_scale(uint8_t fsr);
static float icm45686_convert_temperature(int16_t raw_temp);

static int32_t icm45686_verify_device_id(Icm45686_Handle_t* handle);
static int32_t icm45686_configure_device(Icm45686_Handle_t* handle,
                                         const Icm45686_Config_t* config);

/* ============================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ============================================================================ */

/**
 * @brief Initialize the ICM-45686 sensor
 */
int32_t icm45686_init(Icm45686_Handle_t* handle,
                      SPI_HandleTypeDef* hspi,
                      GPIO_TypeDef* cs_port,
                      uint16_t cs_pin,
                      const Icm45686_Config_t* config)
{
    int32_t status = ICM45686_OK;

    /* Parameter validation */
    if ((handle == NULL) || (hspi == NULL) || (cs_port == NULL) || (config == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Initialize handle structure */
    memset(handle, 0, sizeof(Icm45686_Handle_t));
    handle->hspi = hspi;
    handle->cs_port = cs_port;
    handle->cs_pin = cs_pin;
    handle->is_initialized = false;

    /* Set CS high (idle state) */
    icm45686_cs_high(handle);

    /* Wait for device startup (per datasheet section 4.1) */
    HAL_Delay(ICM45686_STARTUP_TIME_MS);

    /* Verify device ID */
    status = icm45686_verify_device_id(handle);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Perform soft reset to ensure clean state */
    status = icm45686_soft_reset(handle);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Configure device with provided settings */
    status = icm45686_configure_device(handle, config);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Store configuration */
    memcpy(&handle->config, config, sizeof(Icm45686_Config_t));

    /* Calculate scale factors based on configuration */
    handle->accel_scale_factor = icm45686_calculate_accel_scale(config->accel_fsr);
    handle->gyro_scale_factor = icm45686_calculate_gyro_scale(config->gyro_fsr);

    /* Mark as initialized */
    handle->is_initialized = true;

    return ICM45686_OK;
}

/**
 * @brief Read WHO_AM_I register for device identification
 */
int32_t icm45686_read_who_am_i(Icm45686_Handle_t* handle, uint8_t* who_am_i)
{
    if ((handle == NULL) || (who_am_i == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    return icm45686_read_register(handle, ICM45686_WHO_AM_I_REG, who_am_i);
}

/**
 * @brief Read raw sensor data from all axes using DMA
 */
/**
 * @brief Read raw sensor data - CORRECTED for little-endian reception
 */
int32_t icm45686_read_raw_data(Icm45686_Handle_t* handle,
                               Icm45686_RawData_t* data)
{
    int32_t status = ICM45686_OK;
    uint8_t raw_buffer[ICM45686_BURST_READ_SIZE];

    /* Parameter validation */
    if ((handle == NULL) || (data == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Burst read all sensor registers using DMA */
    status = icm45686_read_registers_dma(handle,
                                         ICM45686_REG_ACCEL_DATA_X1_UI,
                                         raw_buffer,
                                         ICM45686_BURST_READ_SIZE);
    if (status != ICM45686_OK) {
        return status;
    }

    /* CORRECTED: Parse as LITTLE-ENDIAN (low byte first) */
    /* If ICM-45686 sends big-endian but your SPI swaps bytes */
    data->accel_x_raw = (int16_t)((raw_buffer[1] << 8) | raw_buffer[0]);  // Swapped
    data->accel_y_raw = (int16_t)((raw_buffer[3] << 8) | raw_buffer[2]);  // Swapped
    data->accel_z_raw = (int16_t)((raw_buffer[5] << 8) | raw_buffer[4]);  // Swapped

    data->gyro_x_raw = (int16_t)((raw_buffer[7] << 8) | raw_buffer[6]);   // Swapped
    data->gyro_y_raw = (int16_t)((raw_buffer[9] << 8) | raw_buffer[8]);   // Swapped
    data->gyro_z_raw = (int16_t)((raw_buffer[11] << 8) | raw_buffer[10]); // Swapped

    data->temp_raw = (int16_t)((raw_buffer[13] << 8) | raw_buffer[12]);   // Swapped

    return ICM45686_OK;
}


/**
 * @brief Read and scale sensor data to engineering units
 */
int32_t icm45686_read_scaled_data(Icm45686_Handle_t* handle,
                                  Icm45686_ScaledData_t* data)
{
    int32_t status = ICM45686_OK;
    Icm45686_RawData_t raw_data;

    /* Parameter validation */
    if ((handle == NULL) || (data == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Read raw sensor data */
    status = icm45686_read_raw_data(handle, &raw_data);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Convert accelerometer to m/s² */
    data->accel_x_g = (float)raw_data.accel_x_raw * handle->accel_scale_factor;
    data->accel_y_g = (float)raw_data.accel_y_raw * handle->accel_scale_factor;
    data->accel_z_g = (float)raw_data.accel_z_raw * handle->accel_scale_factor;

    /* Convert gyroscope to degrees/s */
    data->gyro_x_dps = (float)raw_data.gyro_x_raw * handle->gyro_scale_factor;
    data->gyro_y_dps = (float)raw_data.gyro_y_raw * handle->gyro_scale_factor;
    data->gyro_z_dps = (float)raw_data.gyro_z_raw * handle->gyro_scale_factor;

    /* Convert temperature to degrees Celsius */
    data->temp_degc = icm45686_convert_temperature(raw_data.temp_raw);

    return ICM45686_OK;
}

/**
 * @brief Perform software reset of the device
 */
int32_t icm45686_soft_reset(Icm45686_Handle_t* handle)
{
    int32_t status = ICM45686_OK;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Write soft reset bit to DEVICE_CONFIG register (bank 0, register 0x13) */
    status = icm45686_write_register(handle, 0x13U, 0x01U);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Wait for reset to complete (per datasheet) */
    HAL_Delay(ICM45686_SOFT_RESET_TIME_MS);

    return ICM45686_OK;
}

/**
 * @brief Set accelerometer full-scale range - CORRECTED
 */
int32_t icm45686_set_accel_fsr(Icm45686_Handle_t* handle, uint8_t fsr)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle, ICM45686_REG_ACCEL_CONFIG0, &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Clear FSR bits [6:4] and set new value */
    reg_value = (reg_value & 0x8FU) | (fsr & 0x70U);  /* Mask bits [6:4] */

    /* Write updated configuration */
    status = icm45686_write_register(handle, ICM45686_REG_ACCEL_CONFIG0, reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Update stored configuration and scale factor */
    handle->config.accel_fsr = fsr;
    handle->accel_scale_factor = icm45686_calculate_accel_scale(fsr);

    return ICM45686_OK;
}


/**
 * @brief Set gyroscope full-scale range
 */
int32_t icm45686_set_gyro_fsr(Icm45686_Handle_t* handle, uint8_t fsr)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle, ICM45686_REG_GYRO_CONFIG0, &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Clear FSR bits [7:5] and set new value */
    reg_value = (reg_value & 0x1FU) | (fsr & 0xE0U);

    /* Write updated configuration */
    status = icm45686_write_register(handle, ICM45686_REG_GYRO_CONFIG0, reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Update stored configuration and scale factor */
    handle->config.gyro_fsr = fsr;
    handle->gyro_scale_factor = icm45686_calculate_gyro_scale(fsr);

    return ICM45686_OK;
}

/**
 * @brief Set accelerometer output data rate
 */
int32_t icm45686_set_accel_odr(Icm45686_Handle_t* handle, uint8_t odr)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle, ICM45686_REG_ACCEL_CONFIG0, &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Clear ODR bits [3:0] and set new value */
    reg_value = (reg_value & 0xF0U) | (odr & 0x0FU);

    /* Write updated configuration */
    status = icm45686_write_register(handle, ICM45686_REG_ACCEL_CONFIG0, reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Update stored configuration */
    handle->config.accel_odr = odr;

    return ICM45686_OK;
}

/**
 * @brief Set gyroscope output data rate
 */
int32_t icm45686_set_gyro_odr(Icm45686_Handle_t* handle, uint8_t odr)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle, ICM45686_REG_GYRO_CONFIG0, &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Clear ODR bits [3:0] and set new value */
    reg_value = (reg_value & 0xF0U) | (odr & 0x0FU);

    /* Write updated configuration */
    status = icm45686_write_register(handle, ICM45686_REG_GYRO_CONFIG0, reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Update stored configuration */
    handle->config.gyro_odr = odr;

    return ICM45686_OK;
}

/**
 * @brief Set device power mode
 */
int32_t icm45686_set_power_mode(Icm45686_Handle_t* handle,
                                uint8_t accel_mode,
                                uint8_t gyro_mode)
{
    int32_t status = ICM45686_OK;
    uint8_t pwr_mgmt_value = 0U;

    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Combine accelerometer and gyroscope mode bits */
    pwr_mgmt_value = (accel_mode & 0x03U) | (gyro_mode & 0x0CU);

    /* Write power management register */
    status = icm45686_write_register(handle, ICM45686_REG_PWR_MGMT0, pwr_mgmt_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Wait for mode transition to complete */
    HAL_Delay(ICM45686_MODE_SWITCH_TIME_MS);

    /* Update stored configuration */
    handle->config.accel_mode = accel_mode;
    handle->config.gyro_mode = gyro_mode;

    return ICM45686_OK;
}

/**
 * @brief Perform self-test on accelerometer and gyroscope
 */
int32_t icm45686_self_test(Icm45686_Handle_t* handle,
                           bool* accel_result,
                           bool* gyro_result)
{
    /* Parameter validation */
    if ((handle == NULL) || (accel_result == NULL) || (gyro_result == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Self-test implementation per datasheet section 4.13 */
    /* This is a placeholder - full implementation requires:
     * 1. Read sensor outputs without self-test
     * 2. Enable self-test bits in ACCEL_CONFIG1 and GYRO_CONFIG1
     * 3. Read sensor outputs with self-test
     * 4. Calculate self-test response and compare to factory trim values
     * 5. Disable self-test bits
     */

    *accel_result = true;
    *gyro_result = true;

    return ICM45686_OK;
}

/* ============================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ============================================================================ */

/**
 * @brief Write single register (blocking mode for configuration)
 */
static int32_t icm45686_write_register(Icm45686_Handle_t* handle,
                                       uint8_t reg_addr,
                                       uint8_t value)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    uint8_t tx_data[2];
    uint8_t retry = 0U;

    /* Prepare write transaction: address with write bit, then data */
    tx_data[0] = (reg_addr & 0x7FU);  /* Clear MSB for write */
    tx_data[1] = value;

    /* CS low to start transaction */
    icm45686_cs_low(handle);

    /* Transmit with retry logic */
    for (retry = 0U; retry < ICM45686_MAX_RETRIES; retry++) {
        hal_status = HAL_SPI_Transmit(handle->hspi, tx_data, 2U, ICM45686_SPI_TIMEOUT_MS);
        if (hal_status == HAL_OK) {
            break;
        }
    }

    /* CS high to end transaction */
    icm45686_cs_high(handle);

    /* Small delay for register write to take effect */
    HAL_Delay(1U);

    return (hal_status == HAL_OK) ? ICM45686_OK : ICM45686_ERR_SPI;
}

/**
 * @brief Read single register (blocking mode for configuration)
 */
static int32_t icm45686_read_register(Icm45686_Handle_t* handle,
                                      uint8_t reg_addr,
                                      uint8_t* value)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = (reg_addr | 0x80U);  // Read bit
    tx_data[1] = 0x00U;

    icm45686_cs_low(handle);
    HAL_SPI_TransmitReceive(handle->hspi, tx_data, rx_data, 2U, ICM45686_SPI_TIMEOUT_MS);
    icm45686_cs_high(handle);

    *value = rx_data[1];
    return ICM45686_OK;
}


/**
 * @brief Read multiple registers using DMA for high-speed burst reads
 */
static int32_t icm45686_read_registers_dma(Icm45686_Handle_t* handle,
                                           uint8_t reg_addr,
                                           uint8_t* data,
                                           uint16_t length)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    uint32_t timeout_counter = 0U;
    uint16_t i = 0U;
    uint16_t total_length = 0U;

    if (data == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    /* Calculate total transaction length (address + data bytes) */
    total_length = length + ICM45686_REG_ADDR_SIZE;

    /* Prepare TX buffer: register address with read bit, followed by dummy bytes */
    spi_tx_buffer[0] = (reg_addr | ICM45686_SPI_READ_BIT);
    for (i = 1U; i < total_length; i++) {
        spi_tx_buffer[i] = 0x00U;  /* Dummy bytes for clock generation */
    }

    /* Clear RX buffer */
    memset(spi_rx_buffer, 0, total_length);

    /* Reset DMA completion flag */
    dma_transfer_complete = false;

    /* CS low to start transaction (per datasheet timing requirements) */
    icm45686_cs_low(handle);

    /* Start DMA transaction */
    hal_status = HAL_SPI_TransmitReceive_DMA(handle->hspi,
                                             spi_tx_buffer,
                                             spi_rx_buffer,
                                             total_length);

    if (hal_status != HAL_OK) {
        icm45686_cs_high(handle);
        return ICM45686_ERR_SPI;
    }

    /* Wait for DMA transfer complete with timeout (bounded loop per JPL rules) */
    timeout_counter = 0U;
    while ((dma_transfer_complete == false) && (timeout_counter < ICM45686_SPI_TIMEOUT_MS)) {
        HAL_Delay(1U);
        timeout_counter++;
    }

    /* CS high to end transaction */
    icm45686_cs_high(handle);

    if (timeout_counter >= ICM45686_SPI_TIMEOUT_MS) {
        return ICM45686_ERR_TIMEOUT;
    }

    /* Copy received data (skip first byte which is address echo) */
    memcpy(data, &spi_rx_buffer[1], length);

    return ICM45686_OK;
}

/**
 * @brief Set chip select low
 */
static void icm45686_cs_low(Icm45686_Handle_t* handle)
{
    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);

    /* Small delay to meet CS setup time (tCSS min = 10ns from datasheet) */
    /* At high CPU speeds, explicit delay may be needed */

    for (volatile uint8_t i = 0; i < 5; i++) {
        __NOP();
    }
}

/**
 * @brief Set chip select high
 */
static void icm45686_cs_high(Icm45686_Handle_t* handle)
{
    /* Small delay to meet CS hold time (tCSH min = 10ns from datasheet) */

    HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Calculate accelerometer scale factor (LSB to m/s²)
 */
/**
 * @brief Calculate accelerometer scale factor (LSB to m/s²)
 */
static float icm45686_calculate_accel_scale(uint8_t fsr)
{
    float lsb_per_g = 0.0f;

    /* From datasheet Table 2 (page 19): Sensitivity Scale Factor */
    switch (fsr) {
        case ICM45686_ACCEL_FS_2G:   /* ACCEL_UI_FS_SEL = 4 (0b100) */
            lsb_per_g = 16384.0f;           /* 16,384 LSB/g */
            break;
        case ICM45686_ACCEL_FS_4G:   /* ACCEL_UI_FS_SEL = 3 (0b011) */
            lsb_per_g = 8192.0f;            /* 8,192 LSB/g */
            break;
        case ICM45686_ACCEL_FS_8G:   /* ACCEL_UI_FS_SEL = 2 (0b010) */
            lsb_per_g = 4096.0f;            /* 4,096 LSB/g */
            break;
        case ICM45686_ACCEL_FS_16G:  /* ACCEL_UI_FS_SEL = 1 (0b001) */
            lsb_per_g = 2048.0f;            /* 2,048 LSB/g */
            break;
        case ICM45686_ACCEL_FS_32G:  /* ACCEL_UI_FS_SEL = 0 (0b000) */
            lsb_per_g = 1024.0f;            /* 1,024 LSB/g */
            break;
        default:
            lsb_per_g = 16384.0f;           /* Default to ±2g (most sensitive) */
            break;
    }

    /* Convert to m/s²: (1 / LSB_per_g) * gravity_constant */
    return (1 / lsb_per_g);
}

/**
 * @brief Calculate gyroscope scale factor (LSB to dps)
 */
static float icm45686_calculate_gyro_scale(uint8_t fsr)
{
    float lsb_per_dps = 0.0f;

    /* From datasheet Table 1 (page 18): Sensitivity Scale Factor */
    switch (fsr) {
        case ICM45686_GYRO_FS_15_625DPS:  /* GYRO_UI_FS_SEL = 8 (0b1000) */
            lsb_per_dps = 2097.2f;              /* 2097.2 LSB/dps */
            break;
        case ICM45686_GYRO_FS_31_25DPS:   /* GYRO_UI_FS_SEL = 7 (0b0111) */
            lsb_per_dps = 1048.6f;              /* 1048.6 LSB/dps */
            break;
        case ICM45686_GYRO_FS_62_5DPS:    /* GYRO_UI_FS_SEL = 6 (0b0110) */
            lsb_per_dps = 524.3f;               /* 524.3 LSB/dps */
            break;
        case ICM45686_GYRO_FS_125DPS:     /* GYRO_UI_FS_SEL = 5 (0b0101) */
            lsb_per_dps = 262.0f;               /* 262 LSB/dps */
            break;
        case ICM45686_GYRO_FS_250DPS:     /* GYRO_UI_FS_SEL = 4 (0b0100) */
            lsb_per_dps = 131.0f;               /* 131 LSB/dps */
            break;
        case ICM45686_GYRO_FS_500DPS:     /* GYRO_UI_FS_SEL = 3 (0b0011) */
            lsb_per_dps = 65.5f;                /* 65.5 LSB/dps */
            break;
        case ICM45686_GYRO_FS_1000DPS:    /* GYRO_UI_FS_SEL = 2 (0b0010) */
            lsb_per_dps = 32.8f;                /* 32.8 LSB/dps */
            break;
        case ICM45686_GYRO_FS_2000DPS:    /* GYRO_UI_FS_SEL = 1 (0b0001) */
            lsb_per_dps = 16.4f;                /* 16.4 LSB/dps */
            break;
        case ICM45686_GYRO_FS_4000DPS:    /* GYRO_UI_FS_SEL = 0 (0b0000) */
            lsb_per_dps = 8.2f;                 /* 8.2 LSB/dps */
            break;
        default:
            lsb_per_dps = 16.4f;                /* Default to ±2000dps */
            break;
    }

    /* Convert to dps: (1 / LSB_per_dps) */
    return (1.0f / lsb_per_dps);
}


/**
 * @brief Convert raw temperature to degrees Celsius
 */
static float icm45686_convert_temperature(int16_t raw_temp)
{
    float temp_degc = 0.0f;

    /* Temperature formula from datasheet section 17.14:
     * Temperature (C) = (TEMP_DATA / 128) + 25 */
    temp_degc = ((float)raw_temp / ICM45686_TEMP_SENSITIVITY) + ICM45686_TEMP_OFFSET;

    return temp_degc;
}

/**
 * @brief Verify device ID matches expected value
 */
static int32_t icm45686_verify_device_id(Icm45686_Handle_t* handle)
{
    int32_t status = ICM45686_OK;
    uint8_t who_am_i = 0U;

    status = icm45686_read_who_am_i(handle, &who_am_i);
    if (status != ICM45686_OK) {
        return status;
    }

    if (who_am_i != ICM45686_WHO_AM_I_VALUE) {
        return ICM45686_ERR_WHO_AM_I;
    }

    return ICM45686_OK;
}

/**
 * @brief Configure device with provided settings
 */
static int32_t icm45686_configure_device(Icm45686_Handle_t* handle,
                                         const Icm45686_Config_t* config)
{
    int32_t status = ICM45686_OK;

    /* Set power mode */
    status = icm45686_set_power_mode(handle, config->accel_mode, config->gyro_mode);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Set accelerometer FSR */
    status = icm45686_set_accel_fsr(handle, config->accel_fsr);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Set gyroscope FSR */
    status = icm45686_set_gyro_fsr(handle, config->gyro_fsr);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Set accelerometer ODR */
    status = icm45686_set_accel_odr(handle, config->accel_odr);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Set gyroscope ODR */
    status = icm45686_set_gyro_odr(handle, config->gyro_odr);
    if (status != ICM45686_OK) {
        return status;
    }

    return ICM45686_OK;
}

/**
 * @brief Configure INT1 interrupt sources
 */
int32_t icm45686_config_int1(Icm45686_Handle_t* handle, uint8_t int_config)
{
    int32_t status = ICM45686_OK;

    /* Parameter validation */
    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Write interrupt configuration to INT1_CONFIG0 register */
    status = icm45686_write_register(handle,
                                     ICM45686_REG_INT1_CONFIG0,
                                     int_config);

    return status;
}

/**
 * @brief Enable specific INT1 interrupt source
 */
int32_t icm45686_enable_int1(Icm45686_Handle_t* handle, uint8_t int_mask)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    /* Parameter validation */
    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle,
                                    ICM45686_REG_INT1_CONFIG0,
                                    &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Set specified interrupt enable bits */
    reg_value = (reg_value | int_mask);

    /* Write updated configuration */
    status = icm45686_write_register(handle,
                                     ICM45686_REG_INT1_CONFIG0,
                                     reg_value);

    return status;
}

/**
 * @brief Disable specific INT1 interrupt source
 */
int32_t icm45686_disable_int1(Icm45686_Handle_t* handle, uint8_t int_mask)
{
    int32_t status = ICM45686_OK;
    uint8_t reg_value = 0U;

    /* Parameter validation */
    if (handle == NULL) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Read current configuration */
    status = icm45686_read_register(handle,
                                    ICM45686_REG_INT1_CONFIG0,
                                    &reg_value);
    if (status != ICM45686_OK) {
        return status;
    }

    /* Clear specified interrupt enable bits */
    reg_value = (reg_value & (~int_mask));

    /* Write updated configuration */
    status = icm45686_write_register(handle,
                                     ICM45686_REG_INT1_CONFIG0,
                                     reg_value);

    return status;
}

/**
 * @brief Read INT1 status register
 */
int32_t icm45686_read_int1_status(Icm45686_Handle_t* handle, uint8_t* status_flags)
{
    int32_t status = ICM45686_OK;

    /* Parameter validation */
    if ((handle == NULL) || (status_flags == NULL)) {
        return ICM45686_ERR_INVALID_PARAM;
    }

    if (handle->is_initialized == false) {
        return ICM45686_ERR_NOT_INITIALIZED;
    }

    /* Read interrupt status register */
    status = icm45686_read_register(handle,
                                    ICM45686_REG_INT1_STATUS0,
                                    status_flags);

    return status;
}


/* ============================================================================
 * HAL SPI DMA CALLBACK FUNCTIONS
 * ============================================================================ */

/**
 * @brief SPI transmit-receive DMA complete callback
 * @note This function is called by HAL when DMA transfer completes
 * @note Must be defined in user code to override weak HAL implementation
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    /* Set completion flag for DMA transaction */
    dma_transfer_complete = true;
}

/**
 * @brief SPI error callback
 * @note Called when SPI or DMA error occurs
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    /* Set completion flag to unblock waiting code */
    dma_transfer_complete = true;

    /* Error handling - could log error code here */
    /* For aerospace applications, increment error counter */
}
