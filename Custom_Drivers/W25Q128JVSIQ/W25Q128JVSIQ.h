#pragma once

/*
 * W25Q128JVSIQ.h
 * Driver for the Winbond W25Q128JV flash chip (16 MB, SPI).
 *
 * =====================================================================
 *  SETUP NEEDED (to be done in CubeMX / the .ioc, not in this driver)
 * =====================================================================
 *
 *  SPI1 settings:
 *    Mode .................. Full-Duplex Master
 *    Hardware NSS Signal ... Disable (we drive CS ourselves with a GPIO)
 *    Data Size ............. 8 Bits
 *    First Bit ............. MSB First
 *    Clock Polarity (CPOL) . Low
 *    Clock Phase (CPHA) .... 1 Edge
 *    Baud Rate ............. anything up to 50 MBits/s (8 MBits/s works fine)
 *
 *  Pins:
 *    PA5 -> SPI1_SCK    goes to flash pin 6 (CLK)
 *    PA6 -> SPI1_MISO   goes to flash pin 2 (DO)
 *    PA7 -> SPI1_MOSI   goes to flash pin 5 (DI)
 *    PA4 -> GPIO_Output goes to flash pin 1 (/CS)
 *           User Label: FLASH_CS, Output level: High
 *           (the label makes CubeMX generate FLASH_CS_Pin / FLASH_CS_GPIO_Port in main.h)
 *
 *  Hardware note: flash pin 3 (/WP) and pin 7 (/HOLD) should be tied to 3.3V.
 *
 *  Build settings (Project Properties -> C/C++ General -> Paths and Symbols):
 *    - Source Location: add the Custom_Drivers folder
 *    - Includes: add "../Custom_Drivers"
 *    Then include this driver with:  #include "W25Q128JVSIQ/W25Q128JVSIQ.h"
 *
 * =====================================================================
 *  HOW THE LOG WORKS
 * =====================================================================
 *
 *  W25Q_LogData() only ever adds data to the end of the log. It never erases
 *  or writes over old data, so every run gets added after the last one.
 *  W25Q_Init() finds where the old data ends when the board starts up.
 *  The only way to clear the log is to call one of the erase functions yourself.
 *
 *  Each log entry is saved as:  [your data][1 byte = length of your data]
 *  The length byte can never be 0xFF (erased flash reads as 0xFF), which is how
 *  W25Q_Init() can tell where the log stops. To read entries back, start at the
 *  end: the last byte is the length, and the data is right before it.
 *
 * =====================================================================
 *  EXAMPLE USAGE (in main.c, after MX_SPI1_Init())
 * =====================================================================
 *
 *  W25Q_Handle flash = { &hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin, 0 };
 *
 *  if (W25Q_Init(&flash) != HAL_OK) {
 *      Error_Handler();    // chip not found or not responding
 *  }
 *
 *  // logging a sensor reading
 *  typedef struct {
 *      uint32_t time_ms;
 *      float altitude;
 *      float acceleration;
 *  } FlightSample;
 *
 *  FlightSample sample = { HAL_GetTick(), 120.5f, 9.81f };
 *  W25Q_LogData(&flash, &sample, sizeof(sample));
 *
 *  // reading raw bytes back
 *  uint8_t buffer[64];
 *  W25Q_ReadData(&flash, 0, buffer, sizeof(buffer));
 */

#include "main.h"
#include <stdint.h>

#ifndef HAL_SPI_MODULE_ENABLED
#error "SPI is not enabled. Turn on SPI1 in CubeMX (see the setup notes at the top of W25Q128JVSIQ.h)"
#endif

// Chip size info (from the datasheet)
#define W25Q_PAGE_SIZE      256                 // bytes you can write in one go
#define W25Q_SECTOR_SIZE    4096                // smallest area you can erase
#define W25Q_BLOCK_SIZE     65536
#define W25Q_TOTAL_SIZE     (16UL * 1024 * 1024)
#define W25Q_TOTAL_PAGES    (W25Q_TOTAL_SIZE / W25Q_PAGE_SIZE)

#define W25Q_EXPECTED_ID    0xEF4018            // manufacturer + chip ID for the W25Q128JV
#define W25Q_MAX_LOG_ENTRY  254                 // biggest chunk of data W25Q_LogData() takes

typedef struct {
    SPI_HandleTypeDef *spi;         // e.g. &hspi1
    GPIO_TypeDef *cs_port;          // e.g. FLASH_CS_GPIO_Port
    uint16_t cs_pin;                // e.g. FLASH_CS_Pin
    uint32_t next_log_address;      // where the next log entry goes (set by W25Q_Init)
} W25Q_Handle;

// Checks the chip is there and finds the end of the log. Call this once at startup.
HAL_StatusTypeDef W25Q_Init(W25Q_Handle *flash);

// Reads the 3 byte ID of the chip (should be W25Q_EXPECTED_ID)
HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle *flash, uint32_t *id);

// Reads any amount of data starting at any address
HAL_StatusTypeDef W25Q_ReadData(W25Q_Handle *flash, uint32_t address, uint8_t *buffer, uint32_t length);

// Writes any amount of data starting at any address.
// Flash can only be written once after an erase, so the area must be erased (all 0xFF) first.
HAL_StatusTypeDef W25Q_WriteData(W25Q_Handle *flash, uint32_t address, const uint8_t *data, uint32_t length);

// Erase functions. These are never called by the log code, only use them on purpose!
// The address can be anywhere inside the sector/block you want to erase.
HAL_StatusTypeDef W25Q_EraseSector(W25Q_Handle *flash, uint32_t address);   // 4 KB, takes up to ~0.4 s
HAL_StatusTypeDef W25Q_EraseBlock(W25Q_Handle *flash, uint32_t address);    // 64 KB, takes up to ~2 s
HAL_StatusTypeDef W25Q_EraseChip(W25Q_Handle *flash);                       // everything, takes up to ~200 s

// Adds an entry (1 to 254 bytes) to the end of the log
HAL_StatusTypeDef W25Q_LogData(W25Q_Handle *flash, const void *data, uint32_t length);
