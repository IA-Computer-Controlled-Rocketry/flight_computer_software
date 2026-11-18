#pragma once
/*
 * W25Q128JV (128 Mbit / 16 MB) SPI NOR flash driver, STM32 HAL, standard SPI.
 *
 * CubeMX SPI setup: Full-Duplex Master, 8-bit, MSB first, CPOL Low / CPHA 1 Edge
 * (Mode 0), NSS = Software, baud <= 50 MHz (limit of the 0x03 Read command).
 * CS pin: GPIO_Output, initial level High.
 *
 * Flash rules: programming can only turn 1 bits into 0 bits. Erase (sets
 * everything to 0xFF) before writing over old data. Smallest erase = 4 KB sector.
 */
#include "main.h"
#include <stddef.h>
#include <stdint.h>

#define W25Q_PAGE_SIZE    256u
#define W25Q_SECTOR_SIZE  4096u
#define W25Q_BLOCK_SIZE   65536u
#define W25Q_FLASH_SIZE   (16UL * 1024UL * 1024UL)
#define W25Q_JEDEC_ID     0xEF4018u   /* Winbond, W25Q128JV-IQ/JQ */

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           log_addr;   /* next free address for W25Q_LogAppend */
} W25Q_t;

/* Waits out any leftover busy op, checks the JEDEC ID, then sets log_addr to
 * the end of the existing log so new runs append after old ones. Never erases. */
HAL_StatusTypeDef W25Q_Init(W25Q_t *dev);
HAL_StatusTypeDef W25Q_ReadJEDECID(W25Q_t *dev, uint32_t *id);

/* Any address, any length (bounded by chip size). */
HAL_StatusTypeDef W25Q_Read(W25Q_t *dev, uint32_t addr, uint8_t *buf, uint32_t len);

/* Any address, any length; splits on 256-byte page boundaries. Target must be erased. */
HAL_StatusTypeDef W25Q_Write(W25Q_t *dev, uint32_t addr, const uint8_t *buf, uint32_t len);

/* Manual use only: the log code never calls these.
 * addr is rounded down by the chip to the start of its sector/block. */
HAL_StatusTypeDef W25Q_EraseSector(W25Q_t *dev, uint32_t addr);   /* 4 KB,  up to 400 ms */
HAL_StatusTypeDef W25Q_EraseBlock64(W25Q_t *dev, uint32_t addr);   /* 64 KB, up to 2 s    */
HAL_StatusTypeDef W25Q_EraseChip(W25Q_t *dev);                     /* 16 MB, up to 200 s  */

/*
 * Append-only log. Each record is stored as [data (1..254 bytes)][len byte].
 * The len byte is never 0xFF, so every 256-byte page holding log data has a
 * non-0xFF byte. That is what lets W25Q_Init find the end of the log exactly
 * (binary search), even when the data itself contains 0xFF bytes.
 * Records are only written into erased space past the end, never over old data.
 * Read records back from the end: last byte = len, data sits just before it.
 */
#define W25Q_LOG_MAX_RECORD 254u
HAL_StatusTypeDef W25Q_LogAppend(W25Q_t *dev, const void *data, uint32_t len);
