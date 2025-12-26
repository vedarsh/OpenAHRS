#include "icm45686.h"

#define SPI_READ_BIT        0x80
#define SPI_TIMEOUT         10U

/* --- Private Functions --- */

static inline void cs_low(const spi_bus_t *bus) {
    HAL_GPIO_WritePin(bus->nss_port, bus->nss_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const spi_bus_t *bus) {
    HAL_GPIO_WritePin(bus->nss_port, bus->nss_pin, GPIO_PIN_SET);
}

static int32_t spi_read(const spi_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= SPI_READ_BIT;
    cs_low(bus);
    if (HAL_SPI_Transmit(bus->hspi, &reg, 1, SPI_TIMEOUT) != HAL_OK) goto err;
    if (HAL_SPI_Receive(bus->hspi, buf, len, SPI_TIMEOUT) != HAL_OK) goto err;
    cs_high(bus);
    return 0;
err:
    cs_high(bus);
    return -1;
}

static int32_t spi_write(const spi_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len) {
    cs_low(bus);
    if (HAL_SPI_Transmit(bus->hspi, &reg, 1, SPI_TIMEOUT) != HAL_OK) goto err;
    if (HAL_SPI_Transmit(bus->hspi, buf, len, SPI_TIMEOUT) != HAL_OK) goto err;
    cs_high(bus);
    return 0;
err:
    cs_high(bus);
    return -1;
}

/* --- Public Functions --- */

int32_t icm45686_read_chip_id(const spi_bus_t *bus) {
    uint8_t id = 0;
    if (spi_read(bus, ICM45686_REG_WHO_AM_I, &id, 1)) return -1;
    // Return ID to user for verification (should be 0x60 or 0xE9)
    return (int32_t)id; 
}

int32_t icm45686_soft_reset(const spi_bus_t *bus) {
    uint8_t reset_val = ICM45686_SOFT_RESET_BIT; 
    if (spi_write(bus, ICM45686_REG_REGMISC2, &reset_val, 1)) return -1;
    HAL_Delay(50); // Mandatory wait
    return 0;
}

int32_t icm45686_configure(const spi_bus_t *bus, uint8_t accel_fsr, uint8_t gyro_fsr, uint8_t odr) {
    uint8_t val;
    int32_t ret = 0;

    // Register 0x16: INT1CONFIG0, Bit 2: INT1_DRDY_EN
    val = 0x04; 
    if (spi_write(bus, ICM45686_REG_INT1_CONFIG0, &val, 1)) ret = -1;

    // 1. Accel Config
    val = accel_fsr | (odr & 0x0F);
    if (spi_write(bus, ICM45686_REG_ACCEL_CONFIG0, &val, 1)) ret = -1;

    // 2. Gyro Config
    val = gyro_fsr | (odr & 0x0F);
    if (spi_write(bus, ICM45686_REG_GYRO_CONFIG0, &val, 1)) ret = -1;

    // 3. Enable Sensors (Low Noise Mode)
    val = ICM45686_PWR_GYRO_MODE_LN | ICM45686_PWR_ACCEL_MODE_LN; 
    if (spi_write(bus, ICM45686_REG_PWR_MGMT0, &val, 1)) ret = -1;
    
    HAL_Delay(50);
    return ret;
}


int32_t icm45686_read_raw_data(const spi_bus_t *bus, icm45686_raw_data_t *data) {
    uint8_t buf[12]; 

    // Burst read 12 bytes from Address 0x00 (ACCEL_DATA_X1)
    if (spi_read(bus, ICM45686_REG_ACCEL_DATA_X1, buf, 12)) return -1;

    // Big Endian Parsing
    data->accel_x = (int16_t)((buf[1] << 8) | buf[0]);
    data->accel_y = (int16_t)((buf[3] << 8) | buf[2]);
    data->accel_z = (int16_t)((buf[5] << 8) | buf[4]);
    data->gyro_x  = (int16_t)((buf[7] << 8) | buf[6]);
    data->gyro_y  = (int16_t)((buf[9] << 8) | buf[8]);
    data->gyro_z  = (int16_t)((buf[11] << 8) | buf[10]);

    // Temp is at 0x0C (not contiguous with GyroZ which ends at 0x0B)
    uint8_t temp_buf[2];
    if (spi_read(bus, ICM45686_REG_TEMP_DATA1, temp_buf, 2)) return -1;
    data->temp = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
    
    return 0;
}

int32_t icm45686_read_sensor_ready(const spi_bus_t *bus) {
    uint8_t status = 0;
    if (spi_read(bus, ICM45686_REG_INT1_STATUS0, &status, 1)) return -1;
    return (status & ICM45686_INT_STATUS_DRDY) ? 1 : 0;
}

void icm45686_convert_data(const icm45686_raw_data_t *raw, icm45686_data_t *out) {

    const float accel_scale = 16384.0f; 
    const float gyro_scale = 16.4f;    // 2000DPS

    // Invert X and Y axes
    out->accel_x_g = ((float)raw->accel_x / accel_scale);
    out->accel_y_g = ((float)raw->accel_y / accel_scale);
    out->accel_z_g =  (float)raw->accel_z / accel_scale;
    
    // Invert Gyro X and Y to match Accel orientation
    out->gyro_x_dps = ((float)raw->gyro_x / gyro_scale);
    out->gyro_y_dps = ((float)raw->gyro_y / gyro_scale);
    out->gyro_z_dps =  (float)raw->gyro_z / gyro_scale;
    
    out->temp_c = ((float)raw->temp / 132.48f) + 25.0f;
}
