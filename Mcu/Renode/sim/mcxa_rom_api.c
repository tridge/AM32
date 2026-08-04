/*
 * The MCXA153's flash driver lives in masked boot ROM, reached through
 * a function-pointer tree at 0x03003FE0 (Mcu/a153/Inc/mcxa153_rom_api.h).
 * There is no memory-mapped flash controller to model: the emulation
 * must provide CALLABLE code. This file is compiled by gen_target.py
 * with the same arm toolchain that builds the firmware, linked at the
 * ROM code base, and loaded into the ROM region; the generator reads
 * the symbol addresses back and writes the API tree and the
 * flash_driver_interface_t table into ROM from the .resc.
 *
 * Renode's MappedMemory is writable, so programming and erasing are
 * plain stores; every verify succeeds by construction.
 */

typedef unsigned int u32;
typedef unsigned char u8;
typedef unsigned long uptr;

int rom_flash_init(void *cfg)
{
    (void)cfg;
    return 0;
}

int rom_flash_erase_sector(void *cfg, u32 addr, u32 len, u32 key)
{
    volatile u8 *p = (volatile u8 *)(uptr)addr;
    u32 i;
    (void)cfg;
    (void)key;
    for (i = 0; i < len; i++) {
        p[i] = 0xFF;
    }
    return 0;
}

int rom_flash_program_page(void *cfg, u32 addr, const u8 *src, u32 len)
{
    volatile u8 *p = (volatile u8 *)(uptr)addr;
    u32 i;
    (void)cfg;
    for (i = 0; i < len; i++) {
        p[i] = src[i];
    }
    return 0;
}

int rom_flash_read(void *cfg, u32 addr, u8 *dst, u32 len)
{
    const u8 *p = (const u8 *)(uptr)addr;
    u32 i;
    (void)cfg;
    for (i = 0; i < len; i++) {
        dst[i] = p[i];
    }
    return 0;
}

/* kFLASH_Property codes from Mcu/a153/Inc/mcxa153_rom_api.h */
int rom_flash_get_property(void *cfg, u32 prop, u32 *value)
{
    (void)cfg;
    switch (prop) {
    case 0x00: /* sector size */
        *value = 8192;
        break;
    case 0x01: /* total size */
        *value = 131072;
        break;
    case 0x04: /* block base address */
        *value = 0;
        break;
    case 0x30: /* page size */
        *value = 128;
        break;
    default:
        *value = 0;
        break;
    }
    return 0;
}

/* every verify_* and ifr_* entry: nothing to check against */
int rom_flash_ok(void)
{
    return 0;
}
