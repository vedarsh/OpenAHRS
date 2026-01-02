#include "mag.h"
#include <string.h>

// Static allocation - preferred for embedded systems
static mag_ctx_t mag_ctx_instance;

mag_ctx_t* create_mag_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin)
{
    mag_ctx_t* ctx = &mag_ctx_instance;
    memset(ctx, 0, sizeof(mag_ctx_t));
    
    // Default configuration per ST recommendations [web:21][web:24]
    ctx->mag_config.odr = DEFAULT_MAG_ODR;
    ctx->mag_config.mode = DEFAULT_MAG_MODE;
    ctx->mag_config.temp_comp_enabled = true;
    ctx->mag_config.low_pass_filter_enabled = false;

    ctx->spi_bus.hspi = hspi;
    ctx->spi_bus.nss_pin = Pin;
    ctx->spi_bus.nss_port = Port;

    ctx->health = SENSOR_NOT_INITIALISED;

    return ctx;
}

sensor_state_t init_mag(mag_ctx_t *ctx)
{
    // Configure SPI mode (4-wire) and disable I2C [web:21]
    if (lis2mdl_set_spi_mode(&ctx->spi_bus, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Check chip ID
    if (lis2mdl_read_chip_id(&ctx->spi_bus) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Soft reset
    if (lis2mdl_soft_reset(&ctx->spi_bus) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Configure SPI mode (4-wire) and disable I2C [web:21]
    if (lis2mdl_set_spi_mode(&ctx->spi_bus, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Enable BDU (Block Data Update) and disable I2C [web:24]
    if (lis2mdl_config_interface(&ctx->spi_bus, true, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Set ODR
    if (lis2mdl_set_odr(&ctx->spi_bus, ctx->mag_config.odr) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Enable temperature compensation (recommended) [web:21][web:24]
    if (lis2mdl_enable_compensation(&ctx->spi_bus, ctx->mag_config.temp_comp_enabled) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Set operating mode (continuous or single)
    if (lis2mdl_set_mode(&ctx->spi_bus, ctx->mag_config.mode) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    return ctx->health = SENSOR_OK;
}

sensor_state_t mag_reconfigure(mag_ctx_t *ctx)
{
    // Soft reset
    if (lis2mdl_soft_reset(&ctx->spi_bus) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Reconfigure with stored settings
    if (lis2mdl_set_spi_mode(&ctx->spi_bus, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (lis2mdl_config_interface(&ctx->spi_bus, true, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (lis2mdl_set_odr(&ctx->spi_bus, ctx->mag_config.odr) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (lis2mdl_enable_compensation(&ctx->spi_bus, ctx->mag_config.temp_comp_enabled) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (lis2mdl_set_mode(&ctx->spi_bus, ctx->mag_config.mode) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    return ctx->health = SENSOR_OK;
}

sensor_state_t mag_read_data(mag_ctx_t *ctx, mag_output_t *output)
{
    lis2mdl_data_t mag_data;
    float temp_c;

    if (lis2mdl_set_spi_mode(&ctx->spi_bus, true) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    // Read magnetometer data
    if (lis2mdl_read_mag(&ctx->spi_bus, &mag_data) != 0)
    {
        return ctx->health = SENSOR_DEGRADED;
    }

    // Read temperature
    if (lis2mdl_read_temperature(&ctx->spi_bus, &temp_c) != 0)
    {
        return ctx->health = SENSOR_DEGRADED;
    }

    // Assign to output structure (correct direction)
    output->mag_x_ut = mag_data.x_ut;
    output->mag_y_ut = mag_data.y_ut;
    output->mag_z_ut = mag_data.z_ut;
    output->temp_c = temp_c;
    
    return ctx->health = SENSOR_OK;
}

sensor_state_t mag_data_ready(mag_ctx_t *ctx)
{
    bool ready = lis2mdl_read_sensor_ready(&ctx->spi_bus);
    
    if (!ready)
    {
        return SENSOR_NOT_INITIALISED;  // Data not ready yet
    }
    
    return SENSOR_OK;
}
