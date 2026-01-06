#include "imu.h"
#include <string.h>

// Static allocation - preferred for embedded systems [web:15][web:17]
static imu_ctx_t imu_ctx_instance;

imu_ctx_t* create_imu_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin)
{
    imu_ctx_t* ctx = &imu_ctx_instance;
    memset(ctx, 0, sizeof(imu_ctx_t));
    
    ctx->imu_config.accel_fsr = DEFAULT_ACCEL_FSR;
    ctx->imu_config.gyro_fsr = DEFAULT_GYRO_FSR;
    ctx->imu_config.odr = DEFAULT_ODR;

    ctx->spi_bus.hspi = hspi;
    ctx->spi_bus.nss_pin = Pin;
    ctx->spi_bus.nss_port = Port;

    ctx->health = SENSOR_NOT_INITIALISED;

    return ctx;
}

sensor_state_t init_imu(imu_ctx_t *ctx)
{
    int32_t chip_id = icm45686_read_chip_id(&ctx->spi_bus);
    
    if (chip_id < 0 || chip_id != ICM45686_CHIP_ID) 
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (icm45686_soft_reset(&ctx->spi_bus) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (icm45686_configure(&ctx->spi_bus,
                           ctx->imu_config.accel_fsr,
                           ctx->imu_config.gyro_fsr,
                           ctx->imu_config.odr) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    return ctx->health = SENSOR_OK;
}

sensor_state_t imu_reconfigure(imu_ctx_t *ctx)
{
    if (icm45686_soft_reset(&ctx->spi_bus) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    if (icm45686_configure(&ctx->spi_bus,
                           ctx->imu_config.accel_fsr,
                           ctx->imu_config.gyro_fsr,
                           ctx->imu_config.odr) != 0)
    {
        return ctx->health = SENSOR_ERROR;
    }

    return ctx->health = SENSOR_OK;
}

sensor_state_t imu_read_data(imu_ctx_t *ctx, imu_output_t *output)
{
    icm45686_raw_data_t raw_data;
    icm45686_data_t converted_data;

    // Check return value: -1 = error, 0 = success
    if (icm45686_read_raw_data(&ctx->spi_bus, &raw_data) != 0)
    {
        return ctx->health = SENSOR_DEGRADED;
    }

    icm45686_convert_data(&raw_data, &converted_data);
    
    output->acc_x_g = converted_data.accel_x_g;
    output->acc_y_g = converted_data.accel_y_g;
    output->acc_z_g = converted_data.accel_z_g;

    output->gyr_x_dps = converted_data.gyro_x_dps;
    output->gyr_y_dps = converted_data.gyro_y_dps;
    output->gyr_z_dps = converted_data.gyro_z_dps;
    
    
    return ctx->health = SENSOR_OK;
}

// Optional: Add sensor ready check wrapper
sensor_state_t imu_data_ready(imu_ctx_t *ctx)
{
    int32_t ready = icm45686_read_sensor_ready(&ctx->spi_bus);
    
    if (ready < 0)
    {
        return ctx->health = SENSOR_DEGRADED;
    }
    
    return ready ? SENSOR_OK : SENSOR_NOT_INITIALISED;
}
