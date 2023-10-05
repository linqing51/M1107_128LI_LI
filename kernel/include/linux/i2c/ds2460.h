#ifndef _LINUX_DS2460_H
#define _LINUX_DS2460_H

#include <linux/types.h>
#include <linux/memory.h>

/*
 ** As seen through Linux I2C, differences between the most common types of I2C
 ** memory include:
 ** - How much memory is available (usually specified in bit)?
 ** - What write page size does it support?
 ** - Special flags (16 bit addresses, read_only, world readable...)?
 **
 ** If you set up a custom eeprom type, please double-check the parameters.
 ** Especially page_size needs extra care, as you risk data loss if your value
 ** is bigger than what the chip actually supports!
 **/
/*
 struct memory_accessor {
         ssize_t (*read)(struct memory_accessor *, char *buf, off_t offset,
                         size_t count);
         ssize_t (*write)(struct memory_accessor *, const char *buf,
                           off_t offset, size_t count);
 };
*/
struct ds2460_platform_data {
    u32     byte_len;       /* size (sum of all addr) */
    u32     expose_start;       /* expose to user start */
    u32     expose_len;     /* expose to user length */
    u16     page_size;      /* for writes */
    u8      flags;
#define DS2460_FLAG_ADDR16  0x80    /* address pointer is 16 bit */
#define DS2460_FLAG_READONLY    0x40    /* sysfs-entry will be read-only */
#define DS2460_FLAG_IRUGO   0x20    /* sysfs-entry will be world-readable */
#define DS2460_FLAG_TAKE8ADDR   0x10    /* take always 8 addresses (24c00) */

    void        (*setup)(struct memory_accessor *, void *context);
    void        *context;
};

ssize_t ds2460_read_generic(u8 *buf, loff_t addr, unsigned len);
ssize_t ds2460_write_generic(u8 *buf, loff_t addr, unsigned len);

#endif /* _LINUX_DS2460_H */
