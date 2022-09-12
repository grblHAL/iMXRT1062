// littlefs HAL for grblHAL VFS
// Most of the code has been exctracted from https://github.com/PaulStoffregen/LittleFS

/* LittleFS for Teensy
 * Copyright (c) 2020, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "driver.h"

#if LITTLEFS_ENABLE

#include "littlefs_hal.h"

#if defined(ARDUINO_TEENSY40)
#define FLASH_SIZE  0x1F0000
#define SECTOR_SIZE 32768
#elif defined(ARDUINO_TEENSY41)
#define FLASH_SIZE  0x7C0000
#define SECTOR_SIZE 65536
#elif defined(ARDUINO_TEENSY_MICROMOD)
#define FLASH_SIZE  0xFC0000
#define SECTOR_SIZE 65536
#endif

#define FS_SIZE (512 * 1024)

#if (FS_SIZE & (SECTOR_SIZE - 1)) || FS_SIZE > (FLASH_SIZE - (512 * 1024))
#error "Illegal littlefs file system size!"
#endif

extern void eepromemu_flash_write (void *addr, const void *data, uint32_t len);
extern void eepromemu_flash_erase_sector (void *addr);
extern void eepromemu_flash_erase_32K_block (void *addr);
extern void eepromemu_flash_erase_64K_block (void *addr);

static uint32_t baseaddr = 0;

extern unsigned long _flashimagelen;

static int t4_hal_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size)
{
    const uint8_t *p = (uint8_t *)(baseaddr + block * c->block_size + offset);

    memcpy(buffer, p, size);

    return LFS_ERR_OK;
}

static int t4_hal_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    uint8_t *p = (uint8_t *)(baseaddr + block * c->block_size + offset);

    eepromemu_flash_write(p, buffer, size);

    return LFS_ERR_OK;
}

static int t4_hal_erase (const struct lfs_config *c, lfs_block_t block)
{
    uint8_t *p = (uint8_t *)(baseaddr + block * c->block_size);

#if SECTOR_SIZE == 4096
    eepromemu_flash_erase_sector(p);
#elif SECTOR_SIZE == 32768
    eepromemu_flash_erase_32K_block(p);
#elif SECTOR_SIZE == 65536
    eepromemu_flash_erase_64K_block(p);
#else
#error "Program SECTOR_SIZE must be 4096, 32768, or 65536"
#endif

    return LFS_ERR_OK;
}

static int t4_hal_sync (const struct lfs_config *c)
{
    (void)c;

    return LFS_ERR_OK;
}

static struct lfs_config t4_cfg = {
    // block device operations
    .read = t4_hal_read,
    .prog = t4_hal_prog,
    .erase = t4_hal_erase,
    .sync = t4_hal_sync,
    // block device configuration
    .read_size = 128,
    .prog_size = 128,
    .block_size = SECTOR_SIZE,
    .block_count = FS_SIZE / SECTOR_SIZE,
    .cache_size = 128,
    .lookahead_size = 128,
    .block_cycles = 800
};

struct lfs_config *t4_littlefs_hal (void)
{
    const uint32_t program_size = (uint32_t)&_flashimagelen;
    if (program_size >= FLASH_SIZE)
        return NULL;

    const uint32_t available_space = FLASH_SIZE - program_size;
    if (FS_SIZE > available_space)
        return NULL;

    baseaddr = 0x60000000 + FLASH_SIZE - FS_SIZE - SECTOR_SIZE;

    return &t4_cfg;
}

#endif // LITTLEFS_ENABLE
