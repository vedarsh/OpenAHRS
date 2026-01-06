#include "lis2mdl.h"

#define SPI_READ_BIT   0x80
#define SPI_AUTO_INC   0x40
#define SPI_TIMEOUT    10U

static inline void cs_low(const spi_bus_t *bus)
{
    HAL_GPIO_WritePin(bus->nss_port, bus->nss_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const spi_bus_t *bus)
{
    HAL_GPIO_WritePin(bus->nss_port, bus->nss_pin, GPIO_PIN_SET);
}

static int32_t spi_read(const spi_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg |= SPI_READ_BIT;

    cs_low(bus);

    if (HAL_SPI_Transmit(bus->hspi, &reg, 1, SPI_TIMEOUT) != HAL_OK)
        goto err;

    if (HAL_SPI_Receive(bus->hspi, buf, len, SPI_TIMEOUT) != HAL_OK)
        goto err;

    cs_high(bus);
    return 0;

err:
    cs_high(bus);
    return -1;
}

static int32_t spi_read_block(const spi_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg |= SPI_READ_BIT | SPI_AUTO_INC;

    cs_low(bus);

    if (HAL_SPI_Transmit(bus->hspi, &reg, 1, SPI_TIMEOUT) != HAL_OK)
        goto err;

    if (HAL_SPI_Receive(bus->hspi, buf, len, SPI_TIMEOUT) != HAL_OK)
        goto err;

    cs_high(bus);
    return 0;

err:
    cs_high(bus);
    return -1;
}

static int32_t spi_write(const spi_bus_t *bus, uint8_t reg, uint8_t *buf, uint16_t len)
{
    cs_low(bus);

    if (HAL_SPI_Transmit(bus->hspi, &reg, 1, SPI_TIMEOUT) != HAL_OK)
        goto err;

    if (HAL_SPI_Transmit(bus->hspi, buf, len, SPI_TIMEOUT) != HAL_OK)
        goto err;

    cs_high(bus);
    return 0;

err:
    cs_high(bus);
    return -1;
}

int32_t lis2mdl_read_chip_id(const spi_bus_t *bus)
{
    uint8_t id = 0;

    if (spi_read(bus, LIS2MDL_REG_WHO_AM_I, &id, 1))
        return -1;

    return (id == LIS2MDL_CHIP_ID) ? 0 : -2;
}

int32_t lis2mdl_soft_reset(const spi_bus_t *bus)
{
    uint8_t cfg = LIS2MDL_CFG_A_SOFT_RST;

    if (spi_write(bus, LIS2MDL_REG_CFG_A, &cfg, 1))
        return -1;

    HAL_Delay(5);
    return 0;
}

int32_t lis2mdl_set_spi_mode(const spi_bus_t *bus, bool spi_4wire)
{
    uint8_t cfg;

    if (spi_read(bus, LIS2MDL_REG_CFG_C, &cfg, 1))
        return -1;

    if (spi_4wire)
        cfg |= LIS2MDL_CFG_C_SPI_4WIRE;
    else
        cfg &= ~LIS2MDL_CFG_C_SPI_4WIRE;

    return spi_write(bus, LIS2MDL_REG_CFG_C, &cfg, 1);
}

int32_t lis2mdl_config_interface(const spi_bus_t *bus, bool bdu, bool i2c_disable)
{
    uint8_t cfg;

    if (spi_read(bus, LIS2MDL_REG_CFG_C, &cfg, 1))
        return -1;

    if (bdu) cfg |= LIS2MDL_CFG_C_BDU;
    else cfg &= ~LIS2MDL_CFG_C_BDU;

    if (i2c_disable) cfg |= LIS2MDL_CFG_C_I2C_DISABLE;
    else cfg &= ~LIS2MDL_CFG_C_I2C_DISABLE;

    return spi_write(bus, LIS2MDL_REG_CFG_C, &cfg, 1);
}

int32_t lis2mdl_set_odr(const spi_bus_t *bus, lis2mdl_odr_t odr)
{
    uint8_t cfg;

    if (spi_read(bus, LIS2MDL_REG_CFG_A, &cfg, 1))
        return -1;

    cfg &= ~LIS2MDL_CFG_A_ODR_MASK;
    cfg |= (odr << LIS2MDL_CFG_A_ODR_SHIFT);

    return spi_write(bus, LIS2MDL_REG_CFG_A, &cfg, 1);
}

int32_t lis2mdl_set_mode(const spi_bus_t *bus, lis2mdl_mode_t mode)
{
    uint8_t cfg;

    if (spi_read(bus, LIS2MDL_REG_CFG_A, &cfg, 1))
        return -1;

    cfg &= ~LIS2MDL_CFG_A_MODE_MASK;
    cfg |= (mode << LIS2MDL_CFG_A_MODE_SHIFT);

    return spi_write(bus, LIS2MDL_REG_CFG_A, &cfg, 1);
}

int32_t lis2mdl_enable_compensation(const spi_bus_t *bus, bool enable)
{
    uint8_t cfg;

    if (spi_read(bus, LIS2MDL_REG_CFG_A, &cfg, 1))
        return -1;

    if (enable) cfg |= LIS2MDL_CFG_A_COMP_TEMP_EN;
    else cfg &= ~LIS2MDL_CFG_A_COMP_TEMP_EN;

    return spi_write(bus, LIS2MDL_REG_CFG_A, &cfg, 1);
}

int32_t lis2mdl_read_raw(const spi_bus_t *bus, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (spi_read_block(bus, LIS2MDL_REG_OUTX_L, buf, 6))
        return -1;

    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);

    return 0;
}

int32_t lis2mdl_read_mag(const spi_bus_t *bus, lis2mdl_data_t *data)
{
    int16_t x, y, z;

    if (lis2mdl_read_raw(bus, &x, &y, &z))
        return -1;

    data->x_ut = x * 0.15f;
    data->y_ut = y * 0.15f;
    data->z_ut = z * 0.15f;

    return 0;
}

int32_t lis2mdl_read_temperature(const spi_bus_t *bus, float *temp_c)
{
    uint8_t buf[2];
    int16_t raw;

    if (spi_read_block(bus, LIS2MDL_REG_TEMP_L, buf, 2))
        return -1;

    raw = (int16_t)((buf[1] << 8) | buf[0]);
    *temp_c = 25.0f + (raw / 8.0f);

    return 0;
}


bool lis2mdl_read_sensor_ready(const spi_bus_t *bus)
{
    uint8_t buf = 0;
    if(spi_read(bus, LIS2MDL_REG_STATUS, &buf, 1) == -1) return false;
    return (buf & LIS2MDL_STATUS_ZYXDA);
}
