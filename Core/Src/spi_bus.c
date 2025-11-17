/**
 * @file spi_bus.c
 * @brief SPI Bus Hardware Abstraction Layer Implementation
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 17, 2025
 */

#include "spi_bus.h"
#include <string.h>  // ⭐ FIX #4

/* Private Variables */
static uint8_t spi_tx_buffer[SPI_BUS_MAX_TRANSFER_SIZE];
static uint8_t spi_rx_buffer[SPI_BUS_MAX_TRANSFER_SIZE];
static volatile bool dma_transfer_complete = false;

/* Private Functions */
static inline void spi_cs_select(Spi_Device_t* device);
static inline void spi_cs_deselect(Spi_Device_t* device);

/* Public Functions */

int32_t spi_bus_init_device(Spi_Device_t* device,
                             SPI_HandleTypeDef* hspi,
                             GPIO_TypeDef* cs_port,
                             uint16_t cs_pin)
{
    if (device == NULL || hspi == NULL || cs_port == NULL) {
        return SPI_BUS_ERR_INVALID_PARAM;
    }

    device->hspi = hspi;
    device->cs_port = cs_port;
    device->cs_pin = cs_pin;
    device->use_auto_increment_bit = false;  // Default: no auto-increment

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return SPI_BUS_OK;
}

int32_t spi_bus_write_register(Spi_Device_t* device,
                                uint8_t reg_addr,
                                uint8_t value)
{
    uint8_t tx_data[2];
    HAL_StatusTypeDef hal_status;

    if (device == NULL) {
        return SPI_BUS_ERR_INVALID_PARAM;
    }

    tx_data[0] = reg_addr & 0x7FU;
    tx_data[1] = value;

    spi_cs_select(device);
    hal_status = HAL_SPI_Transmit(device->hspi, tx_data, 2, SPI_BUS_TIMEOUT_MS);
    spi_cs_deselect(device);

    HAL_Delay(1U);
    return (hal_status == HAL_OK) ? SPI_BUS_OK : SPI_BUS_ERR_HAL;
}

int32_t spi_bus_read_register(Spi_Device_t* device,
                               uint8_t reg_addr,
                               uint8_t* value)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    HAL_StatusTypeDef hal_status;

    if (device == NULL || value == NULL) {
        return SPI_BUS_ERR_INVALID_PARAM;
    }

    tx_data[0] = reg_addr | 0x80U;
    tx_data[1] = 0x00U;

    spi_cs_select(device);
    hal_status = HAL_SPI_TransmitReceive(device->hspi, tx_data, rx_data, 2, SPI_BUS_TIMEOUT_MS);
    spi_cs_deselect(device);

    *value = rx_data[1];
    return (hal_status == HAL_OK) ? SPI_BUS_OK : SPI_BUS_ERR_HAL;
}

int32_t spi_bus_read_registers(Spi_Device_t* device,
                                uint8_t reg_addr,
                                uint8_t* data,
                                uint16_t length)
{
    HAL_StatusTypeDef hal_status;
    uint16_t total_length = length + 1U;

    if (device == NULL || data == NULL || length == 0 ||
        total_length > SPI_BUS_MAX_TRANSFER_SIZE) {
        return SPI_BUS_ERR_INVALID_PARAM;
    }

    /* ⭐ FIX #1: Use sensor-specific addressing */
    if (device->use_auto_increment_bit) {
        spi_tx_buffer[0] = reg_addr | 0x80U | 0x40U;  // LIS2MDL style
    } else {
        spi_tx_buffer[0] = reg_addr | 0x80U;          // ICM-45686 style
    }

    memset(&spi_tx_buffer[1], 0x00, length);
    memset(spi_rx_buffer, 0, total_length);

    dma_transfer_complete = false;

    spi_cs_select(device);

    hal_status = HAL_SPI_TransmitReceive_DMA(device->hspi, spi_tx_buffer,
                                             spi_rx_buffer, total_length);

    if (hal_status != HAL_OK) {
        spi_cs_deselect(device);
        return SPI_BUS_ERR_HAL;
    }

    int32_t status = spi_bus_wait_transfer_complete(SPI_BUS_TIMEOUT_MS);

    spi_cs_deselect(device);

    if (status == SPI_BUS_OK) {
        memcpy(data, &spi_rx_buffer[1], length);
    }

    return status;
}

bool spi_bus_is_transfer_complete(void)
{
    return dma_transfer_complete;
}

int32_t spi_bus_wait_transfer_complete(uint32_t timeout_ms)
{
    uint32_t timeout_counter = 0U;

    while (!dma_transfer_complete && (timeout_counter < timeout_ms)) {
        HAL_Delay(1U);
        timeout_counter++;
    }

    return (timeout_counter < timeout_ms) ? SPI_BUS_OK : SPI_BUS_ERR_TIMEOUT;
}

/* Private Functions */

static inline void spi_cs_select(Spi_Device_t* device)
{
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
    for (volatile uint8_t i = 0; i < 5; i++) {
        __NOP();
    }
}

static inline void spi_cs_deselect(Spi_Device_t* device)
{
    for (volatile uint8_t i = 0; i < 5; i++) {
        __NOP();
    }
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
}

/* HAL Callbacks */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_transfer_complete = true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_transfer_complete = true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_transfer_complete = true;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    dma_transfer_complete = true;
}
