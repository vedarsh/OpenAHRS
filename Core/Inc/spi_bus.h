/**
 * @file spi_bus.h
 * @brief SPI Bus Hardware Abstraction Layer
 *
 * @author Vedarsh Reddy Muniratnam
 * @date November 17, 2025
 *
 * @note Provides hardware-independent SPI interface for sensor drivers
 * @note Implements DMA-based high-speed transfers with timeout protection
 */

#ifndef SPI_BUS_H
#define SPI_BUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

/**
 * @brief SPI device handle structure
 * @note Each sensor gets its own device handle with unique CS pin
 */
/**
 * @brief SPI device handle structure
 */
typedef struct {
    SPI_HandleTypeDef* hspi;        /**< Pointer to STM32 SPI handle */
    GPIO_TypeDef* cs_port;          /**< Chip select GPIO port */
    uint16_t cs_pin;                /**< Chip select GPIO pin */
    bool use_auto_increment_bit;    /**< ⭐ ADD THIS LINE! */
} Spi_Device_t;


/**
 * @brief SPI transfer mode
 */
typedef enum {
    SPI_MODE_BLOCKING,              /**< Blocking transfer (polling) */
    SPI_MODE_DMA                    /**< DMA transfer (non-blocking) */
} Spi_TransferMode_t;

/* ============================================================================
 * ERROR CODES
 * ============================================================================ */

#define SPI_BUS_OK                  (0)     /**< Operation successful */
#define SPI_BUS_ERR_TIMEOUT         (-1)    /**< Transfer timeout */
#define SPI_BUS_ERR_INVALID_PARAM   (-2)    /**< Invalid parameter */
#define SPI_BUS_ERR_BUSY            (-3)    /**< Bus is busy */
#define SPI_BUS_ERR_HAL             (-4)    /**< HAL error */

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

#define SPI_BUS_TIMEOUT_MS          (100U)  /**< Default timeout in milliseconds */
#define SPI_BUS_MAX_TRANSFER_SIZE   (256U)  /**< Maximum transfer size */

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Initialize SPI device handle
 * @param[out] device Pointer to device handle structure
 * @param[in] hspi Pointer to configured SPI handle
 * @param[in] cs_port Chip select GPIO port
 * @param[in] cs_pin Chip select GPIO pin
 * @return SPI_BUS_OK on success, negative error code on failure
 */
int32_t spi_bus_init_device(Spi_Device_t* device,
                             SPI_HandleTypeDef* hspi,
                             GPIO_TypeDef* cs_port,
                             uint16_t cs_pin);

/**
 * @brief Write data to SPI device
 * @param[in] device Pointer to device handle
 * @param[in] data Pointer to data buffer
 * @param[in] length Number of bytes to write
 * @param[in] mode Transfer mode (blocking or DMA)
 * @return SPI_BUS_OK on success, negative error code on failure
 */
int32_t spi_bus_write(Spi_Device_t* device,
                      const uint8_t* data,
                      uint16_t length,
                      Spi_TransferMode_t mode);

/**
 * @brief Read data from SPI device
 * @param[in] device Pointer to device handle
 * @param[out] data Pointer to receive buffer
 * @param[in] length Number of bytes to read
 * @param[in] mode Transfer mode (blocking or DMA)
 * @return SPI_BUS_OK on success, negative error code on failure
 */
int32_t spi_bus_read(Spi_Device_t* device,
                     uint8_t* data,
                     uint16_t length,
                     Spi_TransferMode_t mode);

/**
 * @brief Transmit and receive data simultaneously (full-duplex)
 * @param[in] device Pointer to device handle
 * @param[in] tx_data Pointer to transmit buffer
 * @param[out] rx_data Pointer to receive buffer
 * @param[in] length Number of bytes to transfer
 * @param[in] mode Transfer mode (blocking or DMA)
 * @return SPI_BUS_OK on success, negative error code on failure
 */
int32_t spi_bus_transfer(Spi_Device_t* device,
                         const uint8_t* tx_data,
                         uint8_t* rx_data,
                         uint16_t length,
                         Spi_TransferMode_t mode);

/**
 * @brief Write register (convenience function)
 * @param[in] device Pointer to device handle
 * @param[in] reg_addr Register address
 * @param[in] value Register value
 * @return SPI_BUS_OK on success, negative error code on failure
 * @note Uses blocking mode by default
 */
int32_t spi_bus_write_register(Spi_Device_t* device,
                                uint8_t reg_addr,
                                uint8_t value);

/**
 * @brief Read register (convenience function)
 * @param[in] device Pointer to device handle
 * @param[in] reg_addr Register address (with read bit set by function)
 * @param[out] value Pointer to store register value
 * @return SPI_BUS_OK on success, negative error code on failure
 * @note Uses blocking mode by default
 */
int32_t spi_bus_read_register(Spi_Device_t* device,
                               uint8_t reg_addr,
                               uint8_t* value);

/**
 * @brief Read multiple registers with DMA (burst read)
 * @param[in] device Pointer to device handle
 * @param[in] reg_addr Starting register address (with read bit set by function)
 * @param[out] data Pointer to receive buffer
 * @param[in] length Number of bytes to read
 * @return SPI_BUS_OK on success, negative error code on failure
 * @note Uses DMA mode automatically
 */
int32_t spi_bus_read_registers(Spi_Device_t* device,
                                uint8_t reg_addr,
                                uint8_t* data,
                                uint16_t length);

/**
 * @brief Check if DMA transfer is complete
 * @return true if transfer complete, false if still in progress
 */
bool spi_bus_is_transfer_complete(void);

/**
 * @brief Wait for DMA transfer completion with timeout
 * @param[in] timeout_ms Timeout in milliseconds
 * @return SPI_BUS_OK on success, SPI_BUS_ERR_TIMEOUT on timeout
 */
int32_t spi_bus_wait_transfer_complete(uint32_t timeout_ms);

#endif /* SPI_BUS_H */
