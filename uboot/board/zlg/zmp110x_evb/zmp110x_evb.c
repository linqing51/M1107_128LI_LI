#include <common.h>
#include <netdev.h>
#include <asm/arch-zmp110x/zmp_l2buf.h>
#include <asm/arch-zmp110x/spl.h>
#include <asm/arch-zmp110x/zmp_gpio.h>
#include <asm-generic/gpio.h>

DECLARE_GLOBAL_DATA_PTR;


#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_HW_WATCHDOG)

//extern void gpio_set_pin_level(int pin, unsigned char level);


void wdt_init(void)
{
    printf("Watchdog Init\n");
	gpio_direction_output(CONFIG_HW_WATCHDOG_GPIO,0);
}


void hw_watchdog_reset(void)
{
	static unsigned char value = 0;
	value = !value;
	gpio_set_pin_level(CONFIG_HW_WATCHDOG_GPIO,value);
}
#endif


#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
    printf("Boot reached stage %d\n", progress);
}
#endif

#define COMP_MODE_ENABLE ((unsigned int)0x0000EAEF)

/*
 * Miscellaneous platform dependent initialisations
 */
int board_early_init_f (void)
{
	l2_init();
//	drv_lcd_init();
	return 0;
}

//extern int check_anywhere(void);

int board_init (void)
{
	unsigned long val = 0;
//	val = REG32(SHARE_PIN_CFG1_REG);
//	val &= ~(0x3<<10);
//	REG32(SHARE_PIN_CFG1_REG) = val;
//	REG32(GPIO_DIR_REG1) |= (1<<27);
//  REG32(GPIO_OUT_REG1) |= (1<<27);
/*
	val = REG32(SHARE_PIN_CFG0_REG);
	val &= ~(0x3<<4);
	REG32(SHARE_PIN_CFG0_REG) = val;
	REG32(GPIO_DIR_REG1) |= (1<<3);
	REG32(GPIO_OUT_REG1) &=~(1<<3);
*/

//    printf("asic_pll_clk is %lu MHz\n",get_asic_pll_clk());
//    printf("vclk is %lu MHz\n",get_vclk());
//    printf("asic_freq is %lu Hz\n",get_asic_freq());

	drv_lcd_init();
#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_HW_WATCHDOG)
    wdt_init();
#endif

//	check_anywhere();
	write_anywhere();
	gpio_set_pin_as_gpio (41);
	gpio_direction_output(41, 0);
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;


	return 0;
}

int misc_init_r (void)
{
#if 0
	setenv("verify", "n");
#endif
	return (0);
}

/******************************
 Routine:
 Description:
******************************/
int dram_init (void)
{
	/* dram_init must store complete ramsize in gd->ram_size
	 * 这里会简单验证DDR空间是否访问正常，并返回可用空间
	 */
	gd->ram_size = get_ram_size((void *)CONFIG_SYS_SDRAM_BASE,
			PHYS_SDRAM_SIZE);

	debug("%s %s, size = %d CONFIG_SYS_SDRAM_BASE = 0x%x , PHYS_SDRAM_SIZE = 0x%x \n",
		 __FILE__, __func__, gd->ram_size, CONFIG_SYS_SDRAM_BASE, PHYS_SDRAM_SIZE);


	return 0;
}

#ifdef CONFIG_CMD_NET
extern int zmp_ethernet_initialize(u8 dev_num, int base_addr);

int board_eth_init(bd_t *bis)
{
	int rc = 0;

#ifdef CONFIG_ZMP_ETHER
	rc = zmp_ethernet_initialize(0, 0x20300000);
#endif

	return rc;
}
#endif
