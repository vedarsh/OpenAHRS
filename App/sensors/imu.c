#include "imu.h"
#include <string.h>

imu_ctx_t* ctx;

imu_ctx_t* create_imu_ctx(SPI_HandleTypeDef *hspi, GPIO_TypeDef* Port, uint16_t Pin)
{
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
    if (icm45686_read_chip_id(&ctx->spi_bus) != ICM45686_CHIP_ID) 
    {
        return ctx->health = SENSOR_ERROR;
    }

    icm45686_soft_reset(&ctx->spi_bus);

    icm45686_configure(&ctx->spi_bus,
                    ctx->imu_config.accel_fsr,
                    ctx->imu_config.gyro_fsr,
                    ctx->imu_config.odr);

    return ctx->health = SENSOR_OK;
}