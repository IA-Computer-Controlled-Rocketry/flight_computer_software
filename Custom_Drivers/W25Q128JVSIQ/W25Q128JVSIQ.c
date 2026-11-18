#include "W25Q128JVSIQ.h"
#include "main.h"
#include <string.h>

#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_SR1       0x05
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_SECTOR_ERASE   0x20
#define CMD_BLOCK64_ERASE  0xD8
#define CMD_CHIP_ERASE     0xC7
#define CMD_JEDEC_ID       0x9F

#define SR1_BUSY           0x01

/* Max times from the datasheet AC table, plus margin for 1 ms HAL tick. */
#define SPI_TIMEOUT_MS         100u
#define PAGE_PROGRAM_MS        5u
#define SECTOR_ERASE_MS        450u
#define BLOCK64_ERASE_MS       2100u
#define CHIP_ERASE_MS          200000u

static void cs_low(W25Q_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static void cs_high(W25Q_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

/* One CS-framed transaction: send hdr, then send len bytes of tx OR receive len bytes into rx. */
static HAL_StatusTypeDef xfer(W25Q_t *d, const uint8_t *hdr, uint16_t hdr_len,
                              const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    cs_low(d);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, (uint8_t *)hdr, hdr_len, SPI_TIMEOUT_MS);
    if (st == HAL_OK && len) {
        st = tx ? HAL_SPI_Transmit(d->hspi, (uint8_t *)tx, len, SPI_TIMEOUT_MS)
                : HAL_SPI_Receive(d->hspi, rx, len, SPI_TIMEOUT_MS);
    }
    cs_high(d);
    return st;
}

static void addr_cmd(uint8_t hdr[4], uint8_t cmd, uint32_t addr)
{
    hdr[0] = cmd;
    hdr[1] = (uint8_t)(addr >> 16);
    hdr[2] = (uint8_t)(addr >> 8);
    hdr[3] = (uint8_t)addr;
}

static int in_range(uint32_t addr, uint32_t len)
{
    return addr < W25Q_FLASH_SIZE && len <= W25Q_FLASH_SIZE - addr;
}

static HAL_StatusTypeDef wait_ready(W25Q_t *d, uint32_t timeout_ms)
{
    uint8_t cmd = CMD_READ_SR1, sr;
    uint32_t start = HAL_GetTick();
    do {
        if (xfer(d, &cmd, 1, NULL, &sr, 1) != HAL_OK) return HAL_ERROR;
        if (!(sr & SR1_BUSY)) return HAL_OK;
    } while (HAL_GetTick() - start < timeout_ms);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef write_enable(W25Q_t *d)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    return xfer(d, &cmd, 1, NULL, NULL, 0);
}

static HAL_StatusTypeDef erase(W25Q_t *d, uint8_t cmd, uint32_t addr, uint32_t timeout_ms)
{
    uint8_t hdr[4];
    addr_cmd(hdr, cmd, addr);
    uint16_t n = (cmd == CMD_CHIP_ERASE) ? 1 : 4;   /* chip erase has no address */
    if (write_enable(d) != HAL_OK || xfer(d, hdr, n, NULL, NULL, 0) != HAL_OK) return HAL_ERROR;
    return wait_ready(d, timeout_ms);
}

HAL_StatusTypeDef W25Q_ReadJEDECID(W25Q_t *dev, uint32_t *id)
{
    uint8_t cmd = CMD_JEDEC_ID, b[3];
    HAL_StatusTypeDef st = xfer(dev, &cmd, 1, NULL, b, 3);
    *id = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return st;
}

/* *last = index of last non-0xFF byte in the page, or -1 if the page is fully erased. */
static HAL_StatusTypeDef page_last_used(W25Q_t *d, uint32_t page, int32_t *last)
{
    uint8_t buf[W25Q_PAGE_SIZE];
    if (W25Q_Read(d, page * W25Q_PAGE_SIZE, buf, W25Q_PAGE_SIZE) != HAL_OK) return HAL_ERROR;
    *last = (int32_t)W25Q_PAGE_SIZE - 1;
    while (*last >= 0 && buf[*last] == 0xFF) (*last)--;
    return HAL_OK;
}

/* Every page with log data has a non-0xFF len byte, so pages go
 * used, used, ..., used, erased, ..., erased. Binary search the boundary. */
static HAL_StatusTypeDef find_log_end(W25Q_t *d)
{
    uint32_t lo = 0, hi = W25Q_FLASH_SIZE / W25Q_PAGE_SIZE;   /* first erased page is in [lo, hi] */
    int32_t last;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (page_last_used(d, mid, &last) != HAL_OK) return HAL_ERROR;
        if (last < 0) hi = mid; else lo = mid + 1;
    }
    d->log_addr = 0;
    if (lo == 0) return HAL_OK;                                 /* empty chip */
    if (page_last_used(d, lo - 1, &last) != HAL_OK) return HAL_ERROR;
    d->log_addr = (lo - 1) * W25Q_PAGE_SIZE + (uint32_t)last + 1;
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_Init(W25Q_t *dev)
{
    uint32_t id;
    cs_high(dev);
    /* MCU may have reset mid-erase; let the chip finish first. Capped so a
     * missing chip (MISO floats high = BUSY) fails in ~2 s instead of 200 s. */
    if (wait_ready(dev, BLOCK64_ERASE_MS) != HAL_OK) return HAL_ERROR;
    if (W25Q_ReadJEDECID(dev, &id) != HAL_OK) return HAL_ERROR;
    if (id != W25Q_JEDEC_ID) return HAL_ERROR;
    return find_log_end(dev);
}

HAL_StatusTypeDef W25Q_Read(W25Q_t *dev, uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!in_range(addr, len)) return HAL_ERROR;
    while (len) {
        /* Chunked so each HAL call fits its timeout even at slow SPI clocks. */
        uint16_t chunk = len > W25Q_SECTOR_SIZE ? W25Q_SECTOR_SIZE : (uint16_t)len;
        uint8_t hdr[4];
        addr_cmd(hdr, CMD_READ_DATA, addr);
        if (xfer(dev, hdr, 4, NULL, buf, chunk) != HAL_OK) return HAL_ERROR;
        addr += chunk; buf += chunk; len -= chunk;
    }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_Write(W25Q_t *dev, uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!in_range(addr, len)) return HAL_ERROR;
    while (len) {
        /* A page program wraps inside its 256-byte page, so never cross one. */
        uint32_t chunk = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
        if (chunk > len) chunk = len;
        uint8_t hdr[4];
        addr_cmd(hdr, CMD_PAGE_PROGRAM, addr);
        if (write_enable(dev) != HAL_OK
            || xfer(dev, hdr, 4, buf, NULL, (uint16_t)chunk) != HAL_OK
            || wait_ready(dev, PAGE_PROGRAM_MS) != HAL_OK) return HAL_ERROR;
        addr += chunk; buf += chunk; len -= chunk;
    }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_EraseSector(W25Q_t *dev, uint32_t addr)
{
    if (!in_range(addr, 1)) return HAL_ERROR;
    return erase(dev, CMD_SECTOR_ERASE, addr, SECTOR_ERASE_MS);
}

HAL_StatusTypeDef W25Q_EraseBlock64(W25Q_t *dev, uint32_t addr)
{
    if (!in_range(addr, 1)) return HAL_ERROR;
    return erase(dev, CMD_BLOCK64_ERASE, addr, BLOCK64_ERASE_MS);
}

HAL_StatusTypeDef W25Q_EraseChip(W25Q_t *dev)
{
    dev->log_addr = 0;
    return erase(dev, CMD_CHIP_ERASE, 0, CHIP_ERASE_MS);
}

HAL_StatusTypeDef W25Q_LogAppend(W25Q_t *dev, const void *data, uint32_t len)
{
    uint8_t rec[W25Q_LOG_MAX_RECORD + 1];
    if (len == 0 || len > W25Q_LOG_MAX_RECORD) return HAL_ERROR;
    memcpy(rec, data, len);
    rec[len] = (uint8_t)len;                    /* trailer, never 0xFF */
    /* W25Q_Write bounds-checks, so a full chip returns HAL_ERROR, never wraps. */
    HAL_StatusTypeDef st = W25Q_Write(dev, dev->log_addr, rec, len + 1);
    if (st == HAL_OK) dev->log_addr += len + 1;
    return st;
}
