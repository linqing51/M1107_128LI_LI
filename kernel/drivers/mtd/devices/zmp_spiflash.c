/*
 * ZMP spi flash driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/delay.h>
#include <mach/zmp_types.h>

#include <mach/map.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#define FHA_SUCCESS  				0
#define FHA_FAIL     				-1
#define SPI_FLASH_BIN_PAGE_START 	558
#define FLASH_BUF_SIZE				(32*1024)
#define FLASH_PAGESIZE				256
#define SPI_FLASH_READ				1
#define SPI_FLASH_WRITE				2
#define CONFIG_SPIFLASH_USE_FAST_READ 1

#define OPCODE_WREN  			0x06    /* Write Enable */
#define OPCODE_WRDI     		0x04    /* Write Disable */
#define OPCODE_RDSR1    		0x05    /* Read Status Register1 */
#define OPCODE_RDSR2     		0x35    /* Read Status Register2 */
#define OPCODE_RDSR3     		0x15    /* Read Status Register3 */
#define XMC_OPCODE_RDSR2     	0x09    /* XMC Spi flash Read Status Register2 */
#define XMC_OPCODE_RDSR3     	0x95    /* XMC Spi flash Read Status Register3 */

#define OPCODE_WRSR1          	0x01    /* Write Status Register */
#define OPCODE_WRSR2          	0x31    /* Write Status2 Register eg:gd25q128c*/
#define OPCODE_WRSR3          	0x11    /* Write Status3 Register eg:gd25q128c*/
#define XMC_OPCODE_WRSR3        0xC0    /* XMC Spi flash Write Status3 Register eg:gd25q128c*/

#define OPCODE_NORM_READ     	0x03    /* Read Data Bytes */
#define OPCODE_FAST_READ      	0x0b    /* Read Data Bytes at Higher Speed */
#define OPCODE_FAST_D_READ     	0x3b    /* Read Data Bytes at Dual output */
#define OPCODE_FAST_Q_READ     	0x6b    /* Read Data Bytes at Quad output */
#define OPCODE_FAST_D_IO     	0xbb    /* Read Data Bytes at Dual i/o */
#define OPCODE_FAST_Q_IO     	0xeb    /* Read Data Bytes at Quad i/o */

#define OPCODE_PP            	0x02    /* Page Program */
#define OPCODE_PP_DUAL			0x12	/* Dual Page Program*/
#define OPCODE_PP_QUAD			0x32	/* Quad Page Program*/
#define OPCODE_2IO_PP			0x18	/* 2I/O Page Program (tmp)*/
#define OPCODE_4IO_PP			0x38	/* 4I/O Page Program*/

#define OPCODE_BE_4K         	0x20    /* Sector (4K) Erase */
#define OPCODE_BE_32K       	0x52    /* Block (32K) Erase */
#define OPCODE_BE_64K          	0xd8    /* Block (64K) Erase */
#define	OPCODE_SE				0xd8	/* Sector erase (usually 64KiB) */
#define OPCODE_CHIP_ERASE     	0xc7    /* Chip Erase */
#define	OPCODE_RDID				0x9f	/* Read JEDEC ID */
#define OPCODE_DP           	0xb9    /* Deep Power-down */
#define OPCODE_RES          	0xab    /* Release from DP, and Read Signature */

/*
* Used for SST flashes only.
*/
#define	OPCODE_BP				0x02	/* Byte program */
#define	OPCODE_WRDI				0x04	/* Write disable */
#define	OPCODE_AAI_WP			0xad	/* Auto address increment word program */

/*
 * Used for ISSI flash
 */
#define OPCODE_RDBR				0x16
#define OPCODE_WRBRV 			0x17
#define OPCODE_EN4B				0xb7
#define OPCODE_EX4B				0x29

#define SPI_STATUS_REG1	1
#define SPI_STATUS_REG2	2
#define SPI_STATUS_REG3	3

/*
* Define max times to check status register before we give up.
*/
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* 40s max chip erase */

#define	CMD_SIZE			(1)
#define ADDR_SIZE			(4)
#define MAX_DUMMY_SIZE		(4)
#define MTD_PART_NAME_LEN 	(4)

#ifdef CONFIG_SPIFLASH_USE_FAST_READ
#define OPCODE_READ 			OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 	1
#else
#define OPCODE_READ 			OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 	0
#endif

#define ALIGN_DOWN(a, b)  (((a) / (b)) * (b))

#define	SFLAG_UNDER_PROTECT			(1<<0)
#define SFLAG_FAST_READ           	(1<<1)
#define SFLAG_AAAI                	(1<<2)
#define SFLAG_COM_STATUS2         	(1<<3)
#define SFLAG_DUAL_IO_READ         	(1<<4)
#define SFLAG_DUAL_READ           	(1<<5)
#define SFLAG_QUAD_IO_READ         	(1<<6)
#define SFLAG_QUAD_READ           	(1<<7)
#define SFLAG_DUAL_IO_WRITE        	(1<<8)
#define SFLAG_DUAL_WRITE          	(1<<9)
#define SFLAG_QUAD_IO_WRITE        	(1<<10)
#define SFLAG_QUAD_WRITE          	(1<<11)
#define SFLAG_SECT_4K       		(1<<12)
#define SFLAG_COM_STATUS3         	(1<<13)
#define SFLAG_QUAD_NO_QE         	(1<<14)
#define SFLAG_WR_STAT_MODE			(1<<15)

struct partitions
{
	char name[MTD_PART_NAME_LEN];
	unsigned long long size;
	unsigned long long offset;
	unsigned int mask_flags;
}__attribute__((packed));

typedef struct
{
    T_U32 BinPageStart; /*bin data start addr*/
    T_U32 PageSize;     /*spi page size*/
    T_U32 PagesPerBlock;/*page per block*/
    T_U32 BinInfoStart;
    T_U32 FSPartStart;
}
T_SPI_BURN_INIT_INFO;

/*
 * SPI device driver setup and teardown
 */
struct flash_info {
	const char		*name;

	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32			jedec_id;
	u16			ext_id;
	u8			id_ver_c;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16			n_sectors;

	/**
	 *  chip character bits:
	 *  bit 0: under_protect flag, the serial flash under protection or not when power on
	 *  bit 1: fast read flag, the serial flash support fast read or not(command 0Bh)
	 *  bit 2: AAI flag, the serial flash support auto address increment word programming
	 *  bit 3: support dual write or no
	 *  bit 4: support dual read or no
	 *  bit 5: support quad write or no
	 *  bit 6: support quad read or no
	 *  bit 7: the second status command (35h) flag,if use 4-wire(quad) mode,the bit must be is enable
	 */
	u16			flags;

	struct device_node *child;
};

struct zmp_spiflash;

/**
* because of some spi flash is difference of status register difinition.
* this structure use mapping the status reg function and corresponding.
*/
struct flash_status_reg
{
	u32		jedec_id;
	u16		ext_id;
	unsigned b_wip:4;		/*write in progress*/
	unsigned b_wel:4;		/*write ebable latch*/
	unsigned b_bp0:4;		/*block protected 0*/
	unsigned b_bp1:4;		/*block protected 1*/
	unsigned b_bp2:4;		/*block protected 2*/
	unsigned b_bp3:4;		/*block protected 3*/
	unsigned b_bp4:4;		/*block protected 4*/
	unsigned b_srp0:4;		/*status register protect 0*/

	unsigned b_srp1:4;		/*status register protect 1*/
	unsigned b_qe:4;		/*quad enable*/
	unsigned b_lb:4;		/*write protect control and status to the security reg.*/
/*
*	unsigned b_reserved0:4;
*	unsigned b_reserved1:4;
*	unsigned b_reserved2:4;
*/
	unsigned b_cmp:4;		/*conjunction bp0-bp4 bit*/
	unsigned b_sus:4;		/*exec an erase/program suspend command*/
	u8	rd_status_cmd[4];
	u8	wr_status_cmd[4];
	u32  wr_status_mode;
	u32  wr_status_flag;
	u32 (*read_sr)(struct zmp_spiflash *);
	int (*write_sr)(struct zmp_spiflash *, u32);
};

struct zmp_spiflash {
	struct spi_device	*spi;
	struct mutex		lock;
	struct flash_info	info;
	struct mtd_info		mtd;
	unsigned			partitioned:1;

	u8		bus_width;
	u8		command[CMD_SIZE + ADDR_SIZE + MAX_DUMMY_SIZE];
	u8		dummy_len;
	u8		addr_size;
	u8		cmd_adde_size;
	u8		erase_opcode;
	u8		tx_opcode;
	u8		rx_opcode;
	u8		txd_bus_width;
	u8		rxd_bus_width;

	u8		txa_bus_width;
	u8		rxa_bus_width;
	struct flash_status_reg stat_reg;
};

#define CONFIG_SECTION_SIZE 4096    	// cdh:add for erase section size
#define CONFIG_PARATION_TAB_SIZE 512    // cdh:add for paration table section size
#define CONFIG_SPIP_START_PAGE 0    	// cdh:add for searching SPIP flag start flash offset address
#define CONFIG_SPIP_PAGE_CNT  3     	// cdh:add for searching SPIP flag
#define CONFIG_PARATION_PAGE_CNT  2     // cdh:add for searching  paration table page cnt

static struct mtd_info *zmp_mtd_info;

char g_spiflash_flag = 0;


static inline int write_enable(struct zmp_spiflash *flash);
static int wait_till_ready(struct zmp_spiflash *flash);
extern int parse_mtd_partitions(struct mtd_info *master, const char *const *types,struct mtd_partition **pparts,struct mtd_part_parser_data *data);
extern int spinand_get_partition_data(char *partition_name, char *data, unsigned long *date_len);

static void spi_flash_addr(struct zmp_spiflash *flash, u32 addr, u8 *cmd)
{
	/* cmd[0] is actual command */
	if (flash->addr_size == 3){
		cmd[1] = (addr >> 16)&0xff;
		cmd[2] = (addr >> 8)&0xff;
		cmd[3] = (addr >> 0)&0xff;
	} else {
		cmd[1] = (addr >> 24)&0xff;
		cmd[2] = (addr >> 16)&0xff;
		cmd[3] = (addr >> 8)&0xff;
		cmd[4] = (addr >> 0)&0xff;
	}
}

/**
* @brief: because of the _read() function call by mtd block layer, the buffer be
* allocate by vmalloc() in mtd layer, spi driver layer may use this buffer that
* intents of use for DMA transfer, so, add this function to transition buffer.
* call this function at before real read/write data.
*
* @author lixinhai
* @date 2013-04-10
* @param[in] *mtd :mtd infor.
* @return:struct zmp_spiflash: zmp_spiflash struct infor
* @retval zmp_spiflash struct infor
*/
static inline struct zmp_spiflash *mtd_to_spiflash(struct mtd_info *mtd)
{
	return container_of(mtd, struct zmp_spiflash, mtd);
}

static u32 zmp_normal_read_sr(struct zmp_spiflash *flash)
{
	ssize_t retval;
	u8 code;
	u32 status;
	u8 st_tmp= 0;

	/*
	* OPCODE_RDSR1:the first read status cmd
	*/
	code = flash->stat_reg.rd_status_cmd[0];
	if((retval = spi_write_then_read(flash->spi, &code, 1, &st_tmp, 1))<0){
		return retval;
	}
	status = st_tmp;
	if(flash->info.flags & SFLAG_COM_STATUS2){
		code = flash->stat_reg.rd_status_cmd[1];
		if((retval = spi_write_then_read(flash->spi, &code, 1, &st_tmp, 1))<0){
			pr_err("%s,line:%d\n", __func__, __LINE__);
			return retval;
		}
		status = (status | (st_tmp << 8));
	}

   	if(flash->info.flags & SFLAG_COM_STATUS3){
		code = flash->stat_reg.rd_status_cmd[2];
		if((retval = spi_write_then_read(flash->spi, &code, 1, &st_tmp, 1))<0){
			return retval;
		}
		status = (status | (st_tmp << 16));
	}

	return status;
}

static int zmp_normal_write_sr(struct zmp_spiflash *flash, u32 val)
{
	int wr_cnt;
	int ret;

	if(flash->stat_reg.wr_status_mode == 0){
		flash->command[0] = flash->stat_reg.wr_status_cmd[0];//OPCODE_WRSR1;
		flash->command[1] = val & 0xff;
		flash->command[2] = (val>>8) &0xff;

		if (flash->info.flags & SFLAG_COM_STATUS2) {
		    wr_cnt = 3;
		} else {
		    wr_cnt = 2;
		}
		return spi_write(flash->spi, flash->command, wr_cnt);
	}else{
		wait_till_ready(flash);
		write_enable(flash);
		flash->command[0] = flash->stat_reg.wr_status_cmd[0]; //OPCODE_WRSR1;
		flash->command[1] = val & 0xff;
		ret = spi_write(flash->spi, flash->command, 2);

		if(flash->stat_reg.wr_status_flag & 0x2){
			wait_till_ready(flash);
			write_enable(flash);
			flash->command[0] = flash->stat_reg.wr_status_cmd[1]; //OPCODE_WRSR2;
			flash->command[1] = (val>>8) &0xff;
			ret |= spi_write(flash->spi, flash->command, 2);
			wait_till_ready(flash);
		}
		return ret;
	}

}

/**
* @brief Read the status register.
*
*  returning its value in the location
* @author SheShaohua
* @date 2012-03-20
* @param[in] spiflash handle.
* @return int
* @retval Return the status register value.
*/
static u32 read_sr(struct zmp_spiflash *flash)
{
	struct flash_status_reg *fsr = &flash->stat_reg;

	if(fsr && fsr->read_sr)
		return fsr->read_sr(flash);

	return -EINVAL;
}

/**
* @brief Write status register
*
*  Write status register 1 byte.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash  spiflash handle.
* @param[in] val  register value to be write.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a negative error code if failed
*/
static int write_sr(struct zmp_spiflash *flash, u32 val)
{
	struct flash_status_reg *fsr = &flash->stat_reg;

	if(fsr && fsr->write_sr)
		return fsr->write_sr(flash, val);

	return -EINVAL;
}

/**
* @brief Set write enable latch.
*
*  Set write enable latch with Write Enable command.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash  spiflash handle.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a negative error code if failed
*/
static inline int write_enable(struct zmp_spiflash *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/**
* @brief Set write disble
*
*  Set write disble instruction to the chip.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	spiflash handle.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a negative error code if failed
*/
static inline int write_disable(struct zmp_spiflash *flash)
{
	u8	code = OPCODE_WRDI;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/**
* @brief Enter 4B mode
*/
static inline int enter_4b(struct zmp_spiflash *flash)
{
	u8	code = OPCODE_EN4B;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/**
* @brief Exit 4B mode.
*/
static inline int exit_4b(struct zmp_spiflash *flash)
{
	u8	code = OPCODE_EX4B;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}

/**
* @brief  Wait for SPI flash ready.
*
*  Service routine to read status register until ready, or timeout occurs.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	spiflash handle.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int wait_till_ready(struct zmp_spiflash *flash)
{
	unsigned long deadline;
	u32 sr;
	struct flash_status_reg *fsr = &flash->stat_reg;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(flash)) < 0)
			break;
		else if (!(sr & (1<<fsr->b_wip)))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

/**
* @brief  enable spi norflash quad 4 wires mode.
*
* enable spi norflash quad 4 wires mode can set  spi norflash spi_wp and spi_hold share pin as data io
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	spiflash handle.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int quad_mode_enable(struct zmp_spiflash *flash)
{
	int ret;
	u32 regval;
	struct flash_status_reg *fsr = &flash->stat_reg;

	ret = wait_till_ready(flash);
	if (ret){
		pr_err("%s,line:%d\n", __func__, __LINE__);
		return -EBUSY;
	}
	ret = write_enable(flash);
	if (ret){
		pr_err("%s,line:%d\n", __func__, __LINE__);
		return -EIO;
	}

	regval = read_sr(flash);
	regval |= 1<<fsr->b_qe;
	write_sr(flash, regval);

	write_disable(flash);

	return 0;
}

static int fix_erase_disturb(struct zmp_spiflash *flash)
{
	int ret;
    unsigned drive_strength;
    u8 read_code;
    u8 write_code;
    u8 st_tmp= 0;

	struct flash_info *info = &flash->info;

	ret = wait_till_ready(flash);
	if (ret){
		pr_err("%s,line:%d\n", __func__, __LINE__);
		return -EBUSY;
	}
	ret = write_enable(flash);
	if (ret){
		pr_err("%s,line:%d\n", __func__, __LINE__);
		return -EIO;
	}

    if(info->jedec_id == 0xef4017 || info->jedec_id == 0xef4018){
        drive_strength = 5;
        read_code = 0x15;
        write_code = 0x11;
    }

    if((ret = spi_write_then_read(flash->spi, &read_code, 1, &st_tmp, 1))<0){
		return ret;
	}


    st_tmp &= ~(0x3<<drive_strength);
    st_tmp |= (0x3<<drive_strength);

    /*For winbond S19 bit.Recovery for Erase while power drop.*/
    if(info->jedec_id == 0xef4017 || info->jedec_id == 0xef4018){
        st_tmp &= ~(0x1<<3);
        st_tmp |= (0x1<<3);
    }

    {
		wait_till_ready(flash);
		write_enable(flash);
		flash->command[0] = write_code;
		flash->command[1] = st_tmp;
		ret |= spi_write(flash->spi, flash->command, 2);
		wait_till_ready(flash);
	}

	write_disable(flash);

	return 0;
}

/**
* @brief   Erase chip
*
*  Erase the whole flash memory.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	spiflash handle.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int erase_chip(struct zmp_spiflash *flash)
{
	pr_debug( "%s: %s %lldKiB\n",
	      dev_name(&flash->spi->dev), __func__,
	      (long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return -EBUSY;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = OPCODE_CHIP_ERASE;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

/**
* @brief  Erase sector
*
*  Erase a sector specialed by user.
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	    spiflash handle.
* @param[in] offset    which is any address within the sector which should be erased.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int erase_sector(struct zmp_spiflash *flash, u32 offset)
{
	pr_debug("%s: %s %dKiB at 0x%08x\n",
			dev_name(&flash->spi->dev), __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return -EBUSY;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	spi_flash_addr(flash, offset, flash->command);

	spi_write(flash->spi, flash->command, flash->cmd_adde_size);

	return 0;
}

/**
* @brief  MTD Erase
*
* Erase an address range on the flash chip.
* @author SheShaohua
* @date 2012-03-20
* @param[in] mtd    mtd info handle.
* @param[in] instr   erase info.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int zmp_spiflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct zmp_spiflash *flash = mtd_to_spiflash(mtd);
	u32 addr,len;
	uint32_t rem;

	pr_debug( "%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&flash->spi->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > mtd->size) {
		pr_err( "zmp_spiflash_erase:instr->addr[0x%llx] + instr->len[%lld] > mtd->size[%lld]\n",
			instr->addr, instr->len, mtd->size );
		return -EINVAL;
	}

	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem != 0) {
		pr_err( "zmp_spiflash_erase:rem!=0 [%u]\n", rem );
		return -EINVAL;
	}

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == mtd->size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}
		/* REVISIT in some cases we could speed up erasing large regions
		 * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
		 * to use "small sector erase", but that's not always optimal.
		 */

		/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/**
* @brief  init spiflash read and write cmd and data bus width info
*
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	    spiflash handle.
* @return int return init success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int init_spiflash_rw_info(struct zmp_spiflash *flash)
{
	/**default param.*/
	flash->rx_opcode = OPCODE_READ;
	flash->rxd_bus_width = SPI_NBITS_SINGLE;
	flash->rxa_bus_width = SPI_NBITS_SINGLE;
	flash->tx_opcode = OPCODE_PP;
	flash->txd_bus_width = SPI_NBITS_SINGLE;
	flash->txa_bus_width = SPI_NBITS_SINGLE;
	flash->dummy_len = 1;

 	if(flash->bus_width & FLASH_BUS_WIDTH_2WIRE){
		if(flash->info.flags & SFLAG_DUAL_READ) {
			flash->rx_opcode = OPCODE_FAST_D_READ;
			flash->rxd_bus_width = SPI_NBITS_DUAL;
			flash->rxa_bus_width = SPI_NBITS_SINGLE;
			flash->dummy_len = 1;
		} else if (flash->info.flags & SFLAG_DUAL_IO_READ) {
			flash->rx_opcode = OPCODE_FAST_D_IO;
			flash->rxd_bus_width = SPI_NBITS_DUAL;
			flash->rxa_bus_width = SPI_NBITS_DUAL;
			flash->dummy_len = 1;
		}

		if(flash->info.flags & SFLAG_DUAL_WRITE) {
			flash->tx_opcode = OPCODE_PP_DUAL;
			flash->txd_bus_width = SPI_NBITS_DUAL;
			flash->txa_bus_width = SPI_NBITS_SINGLE;
		} else if(flash->info.flags & SFLAG_DUAL_IO_WRITE) {
			flash->tx_opcode = OPCODE_2IO_PP;
			flash->txd_bus_width = SPI_NBITS_DUAL;
			flash->txa_bus_width = SPI_NBITS_DUAL;
		}
	}

	if(flash->bus_width & FLASH_BUS_WIDTH_4WIRE){
		if(flash->info.flags & SFLAG_QUAD_READ) {
			flash->rx_opcode = OPCODE_FAST_Q_READ;
			flash->rxd_bus_width = SPI_NBITS_QUAD;
			flash->rxa_bus_width = SPI_NBITS_SINGLE;
			flash->dummy_len = 1;					//cycle = 8
		}else if(flash->info.flags & SFLAG_QUAD_IO_READ){
			flash->rx_opcode = OPCODE_FAST_Q_IO;
			flash->rxd_bus_width = SPI_NBITS_QUAD;
			flash->rxa_bus_width = SPI_NBITS_QUAD;
			flash->dummy_len = 3;					//cycle = 6
		}

		if(flash->info.flags & SFLAG_QUAD_WRITE) {
			flash->tx_opcode = OPCODE_PP_QUAD;
			flash->txd_bus_width = SPI_NBITS_QUAD;
			flash->txa_bus_width = SPI_NBITS_SINGLE;
		}else if(flash->info.flags & SFLAG_QUAD_IO_WRITE) {
			flash->tx_opcode = OPCODE_4IO_PP;
			flash->txd_bus_width = SPI_NBITS_QUAD;
			flash->txa_bus_width = SPI_NBITS_QUAD;
		}
	}

	return 0;
}

/**
* @brief  cfg spi norflash into quad mode
*
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash	    spiflash handle.
* @return int return init success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int zmp_spiflash_cfg_quad_mode(struct zmp_spiflash *flash)
{
	int ret = 0;

	if((flash->bus_width & FLASH_BUS_WIDTH_4WIRE) &&
		(flash->info.flags & (SFLAG_QUAD_WRITE|SFLAG_QUAD_IO_WRITE|
			SFLAG_QUAD_READ|SFLAG_QUAD_IO_READ))) {
		if((flash->info.flags&SFLAG_QUAD_NO_QE) != SFLAG_QUAD_NO_QE){
			ret = quad_mode_enable(flash);
			if(ret) {
				flash->bus_width &= ~FLASH_BUS_WIDTH_4WIRE;
				pr_err("config the spiflash quad enable fail. transfer use 1 wire.\n");
			}
		}
	}else {
		if (flash->info.flags & (SFLAG_QUAD_WRITE|SFLAG_QUAD_IO_WRITE|
			SFLAG_DUAL_READ|SFLAG_DUAL_IO_READ)){
			if((flash->info.flags&SFLAG_QUAD_NO_QE) != SFLAG_QUAD_NO_QE){
			    ret = quad_mode_enable(flash); // cdh: test KH 1 WIRE for pin7 is not spi_hold but reset
			    if(ret) {
					pr_err("config the spiflash quad enable fail.\n");
			    }
             }
        }
	}

	return ret;
}

/**
* @brief  spi norflash read
*
* spi norflash read operation
* @author SheShaohua
* @date 2012-03-20
* @param[in] mtd    mtd info handle.
* @param[in] from   spi norflash read offset.
* @param[in] len 	  read len
* @param[in] retlen  return real read len
* @param[in] buf    read data buffer
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int spiflash_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct zmp_spiflash *flash = mtd_to_spiflash(mtd);
	struct spi_transfer t[3];
	struct spi_message m;
	void *bounce_buf;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	mutex_lock(&flash->lock);

	bounce_buf =  buf;

	t[0].tx_buf = flash->command;
	t[0].len = CMD_SIZE;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = &flash->command[CMD_SIZE];
	t[1].len = flash->addr_size + flash->dummy_len;
	spi_message_add_tail(&t[1], &m);

	t[2].rx_buf = bounce_buf;
	t[2].len = len;
	t[2].cs_change = 0;
	t[2].rx_nbits = flash->rxd_bus_width;
	spi_message_add_tail(&t[2], &m);

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;

	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
		return -EBUSY;
	}

	/* Set up the write data buffer. */
	memset(flash->command, 0, flash->cmd_adde_size);
	flash->command[0] = flash->rx_opcode;
	spi_flash_addr(flash, from, flash->command);

	spi_sync(flash->spi, &m);
	*retlen = m.actual_length - flash->cmd_adde_size - flash->dummy_len;

	mutex_unlock(&flash->lock);

	return 0;
}

/**
* @brief  mtd every time spi norflash read
*
* mtd spi norflash read operation
* @author SheShaohua
* @date 2012-03-20
* @param[in] mtd    mtd info handle.
* @param[in] from   spi norflash read offset.
* @param[in] len 	  read len
* @param[out] retlen  return real read len
* @param[out] buf    read data buffer
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int zmp_spiflash_read(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	int ret = 0;
	size_t rlen = 0;
	u32 xfer_len;
	u32 offset = 0;
	u32 count = len;

	while(count > 0) {
		xfer_len = (count > FLASH_BUF_SIZE) ? FLASH_BUF_SIZE : count;

		if(xfer_len > FLASH_PAGESIZE)
			xfer_len = ALIGN_DOWN(xfer_len, FLASH_PAGESIZE);

		ret = spiflash_read(mtd, from + offset, xfer_len, &rlen, buf + offset);
		if(unlikely(ret)) {
			ret = -EBUSY;
			goto out;
		}

		*retlen += rlen;
		count -= rlen;
		offset += rlen;
	}
out:
	return ret;
}


/**
* @brief   MTD write
*
* Write an address range to the flash chip.
* @author SheShaohua
* @date 2012-03-20
* @param[in] mtd	mtd info handle.
* @param[in] to 	write start address.
* @param[in] len	write length.
* @param[out] retlen  write length at actually.
* @param[in] buf	   the pointer to write data.
* @return int return write success or failed
* @retval returns zero on success
* @retval return a non-zero error code if failed
*/
static int zmp_spiflash_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	u32 page_offset;
	u32 page_size;
	struct zmp_spiflash *flash = mtd_to_spiflash(mtd);
	struct spi_transfer t[3];
	struct spi_message m;
	void *bounce_buf;

	pr_debug( "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "to",
			(u32)to, len);

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > mtd->size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	mutex_lock(&flash->lock);
	bounce_buf = (void *)buf;

	t[0].tx_buf = flash->command;
	t[0].len = CMD_SIZE;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = &flash->command[CMD_SIZE];
	t[1].len = flash->addr_size;
	spi_message_add_tail(&t[1], &m);

	t[2].tx_buf = bounce_buf;
	t[2].cs_change = 0;
	t[2].tx_nbits = flash->txd_bus_width;
	spi_message_add_tail(&t[2], &m);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return 1;
	}

	write_enable(flash);

	/* Set up the opcode in the write buffer. */
	flash->command[0] = flash->tx_opcode;
	spi_flash_addr(flash, to, flash->command);

	/* what page do we start with? */
	page_offset = to % FLASH_PAGESIZE;

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= FLASH_PAGESIZE) {
		t[2].len = len;
		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - flash->cmd_adde_size;
	} else {
		u32 i;
		/* the size of data remaining on the first page */
		page_size = FLASH_PAGESIZE - page_offset;

		t[2].len = page_size;
		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - flash->cmd_adde_size;

		/* write everything in PAGESIZE chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > FLASH_PAGESIZE)
				page_size = FLASH_PAGESIZE;

			/* write the next page to flash */
			spi_flash_addr(flash, to+i, flash->command);

			t[2].tx_buf = buf + i;
			t[2].len = page_size;

			wait_till_ready(flash);

			write_enable(flash);

			spi_sync(flash->spi, &m);

			if (retlen)
				*retlen += m.actual_length - flash->cmd_adde_size;
		}
	}

	pr_debug("zmp_spiflash_write: retlen=%d\n", *retlen);

	mutex_unlock(&flash->lock);

	return 0;
}

/**
* @brief	MTD get device ID
*
* get the device ID of  the spi flash chip.
* @author SheShaohua
* @date 2012-03-20
* @param[in] mtd	 mtd info handle.
* @return int return device ID of  the spi flash chip.
*/
static int zmp_spiflash_get_devid(struct mtd_info *mtd)
{
	struct zmp_spiflash *flash = mtd_to_spiflash(mtd);
	int			ret;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return -EBUSY;

	/*
	* JEDEC also defines an optional "extended device information"
	* string for after vendor-specific data, after the three bytes
	* we use here.  Supporting some chips might require using it.
	*/
	ret = spi_write_then_read(flash->spi, &code, 1, id, 3);
	if (ret < 0) {
		pr_debug( "%s: error %d zmp_spiflash_get_devid\n",
			dev_name(&flash->spi->dev), ret);
		return -EINVAL;
	}

	jedec = id[0] | (id[1]<<8) | (id[2]<<16);
	pr_info("spi flash ID: 0x%08x\n", jedec);

	return jedec;
}


 /**
 * @brief	MTD write only for SST spi flash.
 *
 * Write an address range to the flash chip.
 * @author SheShaohua
 * @date 2012-03-20
 * @param[in] mtd	 mtd info handle.
 * @param[in] to	 write start address.
 * @param[in] len	 write length.
 * @param[out] retlen  write length at actually.
 * @param[out] buf		the pointer to write data.
 * @return int return write success or failed
 * @retval returns zero on success
 * @retval return a non-zero error code if failed
 */
static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct zmp_spiflash *flash = mtd_to_spiflash(mtd);
	struct spi_transfer t[2];
	struct spi_message m;
	size_t actual;
	int cmd_sz, ret;

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return 0;

	if (to + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = flash->cmd_adde_size;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	ret = wait_till_ready(flash);
	if (ret)
		goto time_out;

	write_enable(flash);

	actual = to % 2;

	/* Start write from odd address. */
	if (actual) {
		flash->command[0] = OPCODE_BP;
		flash->command[1] = to >> 16;
		flash->command[2] = to >> 8;
		flash->command[3] = to;

		/* write one byte. */
		t[1].len = 1;
		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
		*retlen += m.actual_length - flash->cmd_adde_size;
	}
	to += actual;

	flash->command[0] = OPCODE_AAI_WP;
	flash->command[1] = to >> 16;
	flash->command[2] = to >> 8;
	flash->command[3] = to;

	/* Write out most of the data here. */
	cmd_sz = flash->cmd_adde_size;
	for (; actual < len - 1; actual += 2) {
		t[0].len = cmd_sz;
		/* write two bytes. */
		t[1].len = 2;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
		*retlen += m.actual_length - cmd_sz;
		cmd_sz = 1;
		to += 2;
	}
	write_disable(flash);
	ret = wait_till_ready(flash);
	if (ret)
		goto time_out;

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(flash);
		flash->command[0] = OPCODE_BP;
		flash->command[1] = to >> 16;
		flash->command[2] = to >> 8;
		flash->command[3] = to;
		t[0].len = flash->cmd_adde_size;
		t[1].len = 1;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			goto time_out;
		*retlen += m.actual_length - flash->cmd_adde_size;
		write_disable(flash);
	}

time_out:
	mutex_unlock(&flash->lock);
	return ret;
}

/**
* @brief	partition lib erase callback
*
* partition lib spi norflash erase callback function.
* @author SheShaohua
* @date 2012-03-20
* @param[in] chip_num: chip select index
* @param[in] startblock: erase block index
* @return int
* @retval: succes:FHA_SUCCESS; fail:FHA_FAIL
*/
int zmp_fha_erase_callback(T_U32 chip_num,  T_U32 startblock) // startpage
{
	struct erase_info einfo;

	memset(&einfo, 0, sizeof(struct erase_info));
	einfo.addr = startblock * zmp_mtd_info->erasesize; // startpage *  FLASH_PAGESIZE;
	einfo.len = zmp_mtd_info->erasesize;
	einfo.mtd = zmp_mtd_info;

	if(zmp_spiflash_erase(zmp_mtd_info, &einfo) == 0){
		return FHA_SUCCESS;
	}else {
		pr_err("***erase failed\n");
		return FHA_FAIL;
	}
}

/**
* @brief	partition lib write callback
*
* partition lib spi norflash write callback function.
* @author SheShaohua
* @date 2012-03-20
* @param[in] chip_num: chip select index
* @param[in] page_num: write start page  index
* @param[in] data: write data buffer
* @param[in] data_len: write data len
* @param[in] oob:write oob buffer
* @param[in] oob_len: write oob len
* @param[in] eDataType:write data type
* @return int
* @retval: succes:FHA_SUCCESS; fail:FHA_FAIL
*/
int zmp_fha_write_callback(T_U32 chip_num, T_U32 page_num, const T_U8 *data,
		T_U32 data_len, T_U8 *oob, T_U32 oob_len, T_U32 eDataType)
{
	int ret;
	ssize_t retlen;
	loff_t to = page_num * FLASH_PAGESIZE;

	ret = zmp_spiflash_write(zmp_mtd_info, to, data_len * FLASH_PAGESIZE, &retlen, data);
	if(ret){
		pr_err("%s:%d\n", __func__, __LINE__);
		return FHA_FAIL;
	}

	return FHA_SUCCESS;
}

/**
* @brief	partition lib read callback
*
* partition lib spi norflash read callback function.
* @author SheShaohua
* @date 2012-03-20
* @param[in] chip_num: chip select index
* @param[in] page_num: read start page  index
* @param[in] data: read data buffer
* @param[in] data_len: read data len
* @param[in] oob:read oob buffer
* @param[in] oob_len: read oob len
* @param[in] eDataType:read data type
* @return int
* @retval: succes:FHA_SUCCESS; fail:FHA_FAIL
*/
int zmp_fha_read_callback(T_U32 chip_num, T_U32 page_num, T_U8 *data,
		T_U32 data_len, T_U8 *oob, T_U32 oob_len, T_U32 eDataType)
{
	int ret;
	ssize_t retlen;
	loff_t from = page_num * FLASH_PAGESIZE;

	ret = zmp_spiflash_read(zmp_mtd_info, from, data_len * FLASH_PAGESIZE, &retlen, data);
	if (ret) {
		pr_err( "%s:%d\n", __func__, __LINE__);
		return FHA_FAIL;
	}

	return FHA_SUCCESS;
}

/**
* @brief	 spi norfalsh device jedec probe
*
* Read the spi norfalsh device ID and identify that it was supported or not.
* @author SheShaohua
* @date 2012-03-20
* @param[in] spi: spi device handle.
* @return struct flash_info *
* @retval return the device info.
*/
static struct flash_info * jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[5];
	u32			jedec;
	u16         ext_jedec = 0;
	struct device *dev = &spi->dev;
	struct property *prop  = NULL;
	struct flash_info *ofinfo = NULL;
	struct device_node *child;
	u32 readid = 0;
	u32 ext_readid = 0;
	const char *namestr = NULL;
	u32 dataout;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 3);
	if (tmp < 0) {
		pr_debug( "%s: error %d reading JEDEC ID\n",
			dev_name(&spi->dev), tmp);
		return NULL;
	}

	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	pr_err("zmpspi flash ID: 0x%08x\n", jedec);

	/*cdh:search for spi controller device */
	prop = of_find_property(dev->of_node,"compatible", NULL);
	if(!prop){
		pr_err("%s,line:%d, find compatible failed\n", __func__, __LINE__);
		return NULL;
	}

	/*cdh:from norflash dtsi get match spi norflash information */
	namestr = (const char *)kzalloc(20, GFP_KERNEL);
	if (!namestr){
		pr_err("%s,line:%d, allocate name buffer failed\n", __func__, __LINE__);
		return NULL;
	}

	ofinfo = (struct flash_info *)kzalloc(sizeof(struct flash_info), GFP_KERNEL);
	if (!ofinfo){
		pr_err("%s,line:%d, allocate flash info failed\n", __func__, __LINE__);
		kfree(namestr);
		return NULL;
	}

	for_each_available_child_of_node(dev->of_node, child){
		if (child->name && (of_node_cmp(child->name, "spi-norflash") == 0)){
			of_property_read_u32(child, "norflash-jedec-id", &readid);
			if (readid == jedec){
				of_property_read_u32(child, "norflash-ext-id", &ext_readid);
				if (ext_readid != 0 && ext_readid != ext_jedec){
					continue;
				}else{
					ofinfo->child = child;
					break;
				}
			}
		}
	}

	/*
	* cdh:judge search spi norflash device whether success or not
	*/
	if (!ofinfo->child){
		pr_err("%s,line:%d, no find match device!\n", __func__, __LINE__);
		goto err_exit;
	}

	/*
	* cdh:get spi norflash information
	*/
	of_property_read_string(child, "norflash-name", &namestr);
	ofinfo->name = namestr;
	ofinfo->jedec_id = readid;
	ofinfo->ext_id = ext_readid;

	of_property_read_u32(child, "norflash-sector-size", &ofinfo->sector_size);
	of_property_read_u32(child, "norflash-n-sectors", &dataout);
	ofinfo->n_sectors = (u16)dataout;
	of_property_read_u32(child, "norflash-flags", &dataout);
	ofinfo->flags = (u16)dataout;

	return ofinfo;
err_exit:
	kfree(namestr);
	kfree(ofinfo);
	dev_err(&spi->dev, "jedec_probe() unrecognized JEDEC id %06x\n", jedec);
	return NULL;
}

/**
* @brief	 spi norfalsh device init status register
*
* spi norfalsh device init status register
* @author SheShaohua
* @date 2012-03-20
* @param[in] flash: spi flash handle.
* @return int
* @retval: 0: success, othet :failed
*/
static int zmp_spiflash_init_stat_reg(struct zmp_spiflash *flash)
{
	u8			code[5] = {0x5a,0x00,0x00,0x00,0x00}; //OPCODE_VER;
	u8			id[2];
	u8			id_ver_c;
	struct flash_info *info = &flash->info;
	u32 dataout, index;
	int number;
	int ret;

	/*
	*  ����gd25q64 ��B�汾��C �汾��Ȼ�������汾��״̬�Ĵ�����д��һ����������˸Ķ���
	*/
	if(0xc84017 == info->jedec_id){
		ret = spi_write_then_read(flash->spi, code, 5, id, 7);
		if (ret < 0) {
			pr_debug("%s: error %d reading JEDEC VERSION\n",
				__FUNCTION__, ret);
			return 1;
		}
		id_ver_c = id[0];
		if (0x53==id_ver_c){
			info->id_ver_c = 1;
	  	}
	}

	/*
	* search spi norflash status reg info
	*/
	flash->stat_reg.read_sr = zmp_normal_read_sr;
	flash->stat_reg.write_sr = zmp_normal_write_sr;

	/* get dts parameter
	* if has no this property, return -22 less 0, so must judge return value
	* get spi norflash read status reg cmd from dtsi
	*/
	number = of_property_count_u32_elems(info->child, "rd_status_cmd");
	if(number < 0){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}

	for (index = 0; index < number; index++) {
		of_property_read_u32_index(info->child, "rd_status_cmd", index, &dataout);
		flash->stat_reg.rd_status_cmd[index] = dataout;
	}

	/*
	* if has no this property, return -22 less 0, so must judge return value
	* get spi norflash write status reg cmd from dtsi
	*/
	number = of_property_count_u32_elems(info->child, "wr_status_cmd");
	if(number < 0){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}

	for (index = 0; index < number; index++) {
		of_property_read_u32_index(info->child, "wr_status_cmd", index, &dataout);
		flash->stat_reg.wr_status_cmd[index] = (u8)dataout;
	}

	/*
	* get write status reg wr_mode & norflash-wr_flags config information from dtsi
	* wr_mode:0:normal write status reg, 1:one cmd write one status reg, 2:no write cmd
	* norflash-wr_flags: write status reg cnt
	*/
	ret = of_property_read_u32(info->child, "wr_mode", &flash->stat_reg.wr_status_mode);
	if(ret){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}

	if (info->id_ver_c == 1){
		flash->stat_reg.wr_status_mode = 1;
	}

	ret = of_property_read_u32(info->child, "norflash-wr_flags", &flash->stat_reg.wr_status_flag);
	if(ret){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}

	/*
	* get status reg b_wip & b_qe bit map information from dtsi
	*/
	ret = of_property_read_u32(info->child, "norflash-b-wip", &dataout);
	if(ret){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}

	flash->stat_reg.b_wip = dataout;
	ret = of_property_read_u32(info->child, "norflash-b-qe", &dataout);
	if(ret){
		pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
		goto err_int_stat_reg_exit;
	}
	flash->stat_reg.b_qe = dataout;

	return 0;

err_int_stat_reg_exit:
	pr_err("%s,line:%d,Err no this property!!!\n", __func__, __LINE__);
	return -EINVAL;
}
static int spiflash_get_partition_data(char *partition_name, char *data, unsigned long *date_len)
{
	int ret = 0, i = 0;
	struct mtd_partition *real_parts = NULL;
	unsigned long partition_offset = 0;
	unsigned long partition_size = 0;
	size_t retlen = 0;

	if(data == NULL){
		pr_err("data null\n");
		return -1;
	}


	if(partition_name == NULL){
		pr_err("partition_name null\n");
		return -1;
	}
	pr_err("partition name:%s\n",zmp_mtd_info->name);
	pr_err("partition writesize:%d\n",zmp_mtd_info->writesize);
	pr_err("partition erasesize:%d\n",zmp_mtd_info->erasesize);

	ret = parse_mtd_partitions(zmp_mtd_info, NULL, &real_parts, NULL);
	/* Didn't come up with either parsed OR fallback partitions */
	if (ret <= 0) {
		pr_err("failed to find partition %d\n",ret);
		/* Don't abort on errors; we can still use unpartitioned MTD */
		return -1;
	}

	for(i = 0; i < ret; i++){
		if(strncmp(real_parts->name, partition_name, strlen(partition_name)) == 0){

			partition_offset = real_parts->offset;
			partition_size = real_parts->size;
			break;
		}

		pr_info("real_parts->name: %s\n",real_parts->name);
		pr_info("real_parts->offset: %x\n",(uint32_t)real_parts->offset);
		pr_info("real_parts->size: %x\n",(uint32_t)real_parts->size);
		real_parts++;
	}

	if(i == ret){
		pr_err("no find partition %s\n",partition_name);
		return -1;
	}

	if(zmp_spiflash_read(zmp_mtd_info, partition_offset, partition_size, &retlen, data) == -1){
		pr_err("read partition %s data fail\n",partition_name);
		return -1;
	}
	*date_len = retlen;

	return 0;

}

int get_partition_data(char *partition_name, char *data, unsigned long *date_len)
{
	int ret = 0;

	if(g_spiflash_flag){
		ret = spiflash_get_partition_data(partition_name, data, date_len);
	}
	else{
		ret = spinand_get_partition_data(partition_name, data, date_len);
	}
	return ret;
}


EXPORT_SYMBOL_GPL(get_partition_data);


static int  zmp_spiflash_probe(struct spi_device *spi)
{
	struct zmp_spiflash		*flash;
	struct flash_info		*info;
	unsigned i,bus_width = 0;
	u8 code = OPCODE_RDBR, reg;
	int	ret = 0;

    ret = of_property_read_u32(spi->dev.of_node, "bus-width",
				   &bus_width);
	if (ret < 0) {
		 pr_err("Could not read bus-width property\n");
         return -ENODEV;
	}

	info = jedec_probe(spi);
	if (!info){
		pr_err("%s,line:%d, jedec probe device failed!\n", __func__, __LINE__);
		return -ENODEV;
	}
	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash){
		goto err_flash_exit;
	}

	zmp_mtd_info = &flash->mtd;

    /*support spi bus 2 and 4 bits transfer mode. */
    spi->mode =  SPI_TX_QUAD | SPI_RX_QUAD;

	flash->spi = spi;
	flash->info = *info;
	mutex_init(&flash->lock);

	dev_set_drvdata(&spi->dev, flash);
	flash->bus_width = bus_width;
	flash->mtd.name = dev_name(&spi->dev);
	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = FLASH_PAGESIZE;
	flash->mtd.flags = MTD_WRITEABLE;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd._erase = zmp_spiflash_erase;
	flash->mtd._read = zmp_spiflash_read;
	flash->mtd.get_device_id = zmp_spiflash_get_devid;


	flash->addr_size = 3;
	if (flash->mtd.size > SZ_16M) {
		flash->addr_size = 4;
		enter_4b(flash);
		code = OPCODE_RDBR;
		spi_write_then_read(flash->spi, &code, 1, &reg, 1);
		pr_debug("bank_reg[0x%x]\n", reg);
		if (reg & 0x80) {
			pr_debug("enter_4b success reg[0x%x]\n", reg);
		}else {
			pr_err("Failed to enter 4B mode, only the lower 16MiB address can be accessed\n");
			flash->mtd.size = SZ_16M;
			flash->addr_size = 3;
		}
	}
	flash->cmd_adde_size = flash->addr_size + 1;

	/* sst flash chips use AAI word program */
	if ((info->jedec_id >> 16) == 0xbf)
		flash->mtd._write = sst_write;
	else
		flash->mtd._write = zmp_spiflash_write;

	/* prefer "small sector" erase if possible */
	if (info->flags & SFLAG_SECT_4K) {
		flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	/* get spi norflash status reg information from dtsi */
	ret = zmp_spiflash_init_stat_reg(flash);
	if(ret){
		pr_err("%s,line:%d, init stat reg failed!\n", __func__, __LINE__);
		goto err_probe_flash_exit;
	}
	/*
	* atmel serial flash tend to power up
	* with the software protection bits set
	* here has some notes, because now sr completed init
	*/
	if ((info->jedec_id >> 16) == 0x1f) {
		write_enable(flash);
		write_sr(flash, 0);
	}

	/* cfg norflash quad mode */
	ret = zmp_spiflash_cfg_quad_mode(flash);
	if(ret){
		pr_err("%s,line:%d, cfg quad mode failed!\n", __func__, __LINE__);
		goto err_probe_flash_exit;
	}

    /*
    **Only For W25QxxJVxxIQ series chip.
    **For winbond S19 bit.Recovery for Erase while power drop.
    **/
    if(info->jedec_id == 0xef4017 || info->jedec_id == 0xef4018){
        fix_erase_disturb(flash);
    }

	/* init norflash rw bus width cmd */
	init_spiflash_rw_info(flash);

	flash->mtd.dev.parent = &spi->dev;
	dev_info(&spi->dev, "%s (%lld Kbytes)\n", info->name,
			(long long)flash->mtd.size >> 10);

	pr_info("mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	dev_info(&spi->dev, "flash->tx_opcode[0x%x], flash->rx_opcode[0x%x], flash->erase_opcode[0x%x]\n",
		flash->tx_opcode, flash->rx_opcode, flash->erase_opcode);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			pr_debug("mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);
	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	* cdh:must use ,here mount rootfs block
	*/
	ret = mtd_device_parse_register(zmp_mtd_info, NULL, NULL, NULL, 0);
	if (ret) {
		printk("Add root MTD device failed\n");
		goto err_probe_flash_exit;
	}

	g_spiflash_flag = 1;

	return 0;

err_probe_flash_exit:
	kfree(flash);
err_flash_exit:
	kfree(info->name);
	kfree(info);

	return -EINVAL;
}

static int  zmp_spiflash_remove(struct spi_device *spi)
{
	struct zmp_spiflash	*flash = dev_get_drvdata(&spi->dev);
	struct flash_info *info = &flash->info;
	int		status;

	status = mtd_device_unregister(&flash->mtd);
	if (status == 0) {
		kfree(flash);
		kfree(info->name);
		kfree(info);
	}
	return 0;
}

static const struct of_device_id spiflash_of_table[] = {
	{ .compatible = "zlgmcu,zmp-spiflash" },
	{}
};
MODULE_DEVICE_TABLE(of, spiflash_of_table);

static struct spi_driver zmp_spiflash_driver = {
	.driver = {
		.name	= "zmp-spiflash",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = spiflash_of_table,
	},
	.probe	= zmp_spiflash_probe,
	.remove	= zmp_spiflash_remove,
};

static int __init zmp_spiflash_init(void)
{
	return spi_register_driver(&zmp_spiflash_driver);
}

static void __exit zmp_spiflash_exit(void)
{
	spi_unregister_driver(&zmp_spiflash_driver);
}

module_init(zmp_spiflash_init);
module_exit(zmp_spiflash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ZLGMCU Microelectronic Ltd.");
MODULE_DESCRIPTION("ZMP spiflash driver");
MODULE_VERSION("1.0.01");
