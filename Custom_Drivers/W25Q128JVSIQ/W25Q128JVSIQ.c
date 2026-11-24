#include "W25Q128JVSIQ.h"
#include "main.h"
#include <string.h>

// Command bytes (see the "Instructions" section of the W25Q128JV datasheet)
#define CMD_WRITE_ENABLE    0x06
#define CMD_READ_STATUS     0x05
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_BLOCK_ERASE     0xD8
#define CMD_CHIP_ERASE      0xC7
#define CMD_READ_ID         0x9F

#define STATUS_BUSY         0x01    // bit 0 of the status register is 1 while the chip is busy

// Timeouts in ms, a little longer than the worst case times in the datasheet
#define SPI_TIMEOUT             100
#define PAGE_WRITE_TIMEOUT      5
#define SECTOR_ERASE_TIMEOUT    500
#define BLOCK_ERASE_TIMEOUT     2500
#define CHIP_ERASE_TIMEOUT      200000


static void chip_select(W25Q_Handle *flash)
{
    HAL_GPIO_WritePin(flash->cs_port, flash->cs_pin, GPIO_PIN_RESET);
}

static void chip_deselect(W25Q_Handle *flash)
{
    HAL_GPIO_WritePin(flash->cs_port, flash->cs_pin, GPIO_PIN_SET);
}

// Sends a command followed by a 3 byte address (most significant byte first)
static HAL_StatusTypeDef send_command_with_address(W25Q_Handle *flash, uint8_t command, uint32_t address)
{
    uint8_t bytes[4];
    bytes[0] = command;
    bytes[1] = (address >> 16) & 0xFF;
    bytes[2] = (address >> 8) & 0xFF;
    bytes[3] = address & 0xFF;
    return HAL_SPI_Transmit(flash->spi, bytes, 4, SPI_TIMEOUT);
}

// Sends a single command byte on its own (CS goes low, byte, CS goes high)
static HAL_StatusTypeDef send_single_command(W25Q_Handle *flash, uint8_t command)
{
    chip_select(flash);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(flash->spi, &command, 1, SPI_TIMEOUT);
    chip_deselect(flash);
    return result;
}

// The chip has to be told "write enable" before every write or erase
static HAL_StatusTypeDef write_enable(W25Q_Handle *flash)
{
    return send_single_command(flash, CMD_WRITE_ENABLE);
}

// Keeps checking the status register until the chip is done with a write/erase
static HAL_StatusTypeDef wait_until_ready(W25Q_Handle *flash, uint32_t timeout_ms)
{
    uint8_t command = CMD_READ_STATUS;
    uint8_t status = 0;
    uint32_t start_time = HAL_GetTick();

    while (HAL_GetTick() - start_time < timeout_ms) {
        chip_select(flash);
        HAL_StatusTypeDef result = HAL_SPI_Transmit(flash->spi, &command, 1, SPI_TIMEOUT);
        if (result == HAL_OK) {
            result = HAL_SPI_Receive(flash->spi, &status, 1, SPI_TIMEOUT);
        }
        chip_deselect(flash);

        if (result != HAL_OK) {
            return result;
        }
        if ((status & STATUS_BUSY) == 0) {
            return HAL_OK;
        }
    }
    return HAL_TIMEOUT;
}

// Makes sure we don't try to read/write past the end of the chip
static int is_valid_range(uint32_t address, uint32_t length)
{
    return address < W25Q_TOTAL_SIZE && length <= W25Q_TOTAL_SIZE - address;
}


HAL_StatusTypeDef W25Q_ReadID(W25Q_Handle *flash, uint32_t *id)
{
    uint8_t command = CMD_READ_ID;
    uint8_t id_bytes[3] = {0};

    chip_select(flash);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(flash->spi, &command, 1, SPI_TIMEOUT);
    if (result == HAL_OK) {
        result = HAL_SPI_Receive(flash->spi, id_bytes, 3, SPI_TIMEOUT);
    }
    chip_deselect(flash);

    *id = (id_bytes[0] << 16) | (id_bytes[1] << 8) | id_bytes[2];
    return result;
}

HAL_StatusTypeDef W25Q_ReadData(W25Q_Handle *flash, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!is_valid_range(address, length)) {
        return HAL_ERROR;
    }

    // HAL_SPI_Receive can only take 65535 bytes at a time, so big reads are done in 4 KB pieces
    while (length > 0) {
        uint16_t piece = (length > 4096) ? 4096 : length;

        chip_select(flash);
        HAL_StatusTypeDef result = send_command_with_address(flash, CMD_READ_DATA, address);
        if (result == HAL_OK) {
            result = HAL_SPI_Receive(flash->spi, buffer, piece, SPI_TIMEOUT);
        }
        chip_deselect(flash);

        if (result != HAL_OK) {
            return result;
        }
        address += piece;
        buffer += piece;
        length -= piece;
    }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_WriteData(W25Q_Handle *flash, uint32_t address, const uint8_t *data, uint32_t length)
{
    if (!is_valid_range(address, length)) {
        return HAL_ERROR;
    }

    while (length > 0) {
        // One write can't go past the end of a 256 byte page (it would wrap around
        // to the start of the same page), so only write up to the end of this page
        uint32_t space_left_in_page = W25Q_PAGE_SIZE - (address % W25Q_PAGE_SIZE);
        uint32_t piece = (length < space_left_in_page) ? length : space_left_in_page;

        if (write_enable(flash) != HAL_OK) {
            return HAL_ERROR;
        }

        chip_select(flash);
        HAL_StatusTypeDef result = send_command_with_address(flash, CMD_PAGE_PROGRAM, address);
        if (result == HAL_OK) {
            result = HAL_SPI_Transmit(flash->spi, (uint8_t *)data, piece, SPI_TIMEOUT);
        }
        chip_deselect(flash);

        if (result != HAL_OK || wait_until_ready(flash, PAGE_WRITE_TIMEOUT) != HAL_OK) {
            return HAL_ERROR;
        }
        address += piece;
        data += piece;
        length -= piece;
    }
    return HAL_OK;
}

// Shared code for sector and block erase
static HAL_StatusTypeDef erase_area(W25Q_Handle *flash, uint8_t command, uint32_t address, uint32_t timeout_ms)
{
    if (!is_valid_range(address, 1)) {
        return HAL_ERROR;
    }
    if (write_enable(flash) != HAL_OK) {
        return HAL_ERROR;
    }

    chip_select(flash);
    HAL_StatusTypeDef result = send_command_with_address(flash, command, address);
    chip_deselect(flash);

    if (result != HAL_OK) {
        return result;
    }
    return wait_until_ready(flash, timeout_ms);
}

HAL_StatusTypeDef W25Q_EraseSector(W25Q_Handle *flash, uint32_t address)
{
    return erase_area(flash, CMD_SECTOR_ERASE, address, SECTOR_ERASE_TIMEOUT);
}

HAL_StatusTypeDef W25Q_EraseBlock(W25Q_Handle *flash, uint32_t address)
{
    return erase_area(flash, CMD_BLOCK_ERASE, address, BLOCK_ERASE_TIMEOUT);
}

HAL_StatusTypeDef W25Q_EraseChip(W25Q_Handle *flash)
{
    if (write_enable(flash) != HAL_OK || send_single_command(flash, CMD_CHIP_ERASE) != HAL_OK) {
        return HAL_ERROR;
    }
    flash->next_log_address = 0;    // the log is empty now
    return wait_until_ready(flash, CHIP_ERASE_TIMEOUT);
}


// Finds the last byte in a page that isn't 0xFF.
// Sets last_byte to -1 if the whole page is empty (all 0xFF).
static HAL_StatusTypeDef find_last_written_byte(W25Q_Handle *flash, uint32_t page, int *last_byte)
{
    uint8_t page_data[W25Q_PAGE_SIZE];

    if (W25Q_ReadData(flash, page * W25Q_PAGE_SIZE, page_data, W25Q_PAGE_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    *last_byte = W25Q_PAGE_SIZE - 1;
    while (*last_byte >= 0 && page_data[*last_byte] == 0xFF) {
        (*last_byte)--;
    }
    return HAL_OK;
}

// Finds where the old log data ends so new entries go after it.
//
// Every log entry ends with a length byte that is never 0xFF, and no entry is
// longer than a page, so any page with log data in it has at least one byte
// that isn't 0xFF. That means the chip always looks like:
//     [pages with data][empty pages]
// so we can binary search for the first empty page instead of reading all 16 MB.
static HAL_StatusTypeDef find_end_of_log(W25Q_Handle *flash)
{
    uint32_t low = 0;
    uint32_t high = W25Q_TOTAL_PAGES;   // the first empty page is somewhere from low to high
    int last_byte;

    while (low < high) {
        uint32_t middle = (low + high) / 2;

        if (find_last_written_byte(flash, middle, &last_byte) != HAL_OK) {
            return HAL_ERROR;
        }

        if (last_byte == -1) {
            high = middle;      // middle page is empty, so the log ends at or before it
        } else {
            low = middle + 1;   // middle page has data, so the log ends after it
        }
    }

    if (low == 0) {
        flash->next_log_address = 0;    // nothing logged yet
        return HAL_OK;
    }

    // "low" is the first empty page, so the log ends inside the page before it
    if (find_last_written_byte(flash, low - 1, &last_byte) != HAL_OK) {
        return HAL_ERROR;
    }
    flash->next_log_address = (low - 1) * W25Q_PAGE_SIZE + last_byte + 1;
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_Init(W25Q_Handle *flash)
{
    chip_deselect(flash);

    // If the board reset in the middle of an erase, give the chip time to finish.
    // (If the chip isn't connected this also stops us from waiting forever.)
    if (wait_until_ready(flash, BLOCK_ERASE_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    uint32_t id = 0;
    if (W25Q_ReadID(flash, &id) != HAL_OK || id != W25Q_EXPECTED_ID) {
        return HAL_ERROR;
    }

    return find_end_of_log(flash);
}

HAL_StatusTypeDef W25Q_LogData(W25Q_Handle *flash, const void *data, uint32_t length)
{
    uint8_t entry[W25Q_MAX_LOG_ENTRY + 1];

    if (length == 0 || length > W25Q_MAX_LOG_ENTRY) {
        return HAL_ERROR;
    }

    // Entry = the data, then one byte saying how long the data was
    memcpy(entry, data, length);
    entry[length] = length;

    // W25Q_WriteData returns an error if the chip is full, so the log never wraps around
    HAL_StatusTypeDef result = W25Q_WriteData(flash, flash->next_log_address, entry, length + 1);
    if (result == HAL_OK) {
        flash->next_log_address += length + 1;
    }
    return result;
}
