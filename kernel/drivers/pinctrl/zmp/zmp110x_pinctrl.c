/*
 * Driver for ZMP110x GPIO (pinctrl + GPIO)
 *
 * This driver is inspired by:
 * pinctrl-bcm2835.c, please see original file for copyright information
 * pinctrl-tegra.c, please see original file for copyright information
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/bitmap.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/clk.h>

#include <mach/map.h>

#define MODULE_NAME "zmp-pinctrl"
#define ZMP110X_NUM_GPIOS 123
#define ZMP110X_NUM_BANKS 1

#define	ZMP110X_PULLUPDOWN_DISABLE	0
#define	ZMP110X_PULLUPDOWN_ENABLE	1

#define	ePIN_AS_GPIO_IN  0		// pin as gpio in
#define	ePIN_AS_GPIO_OUT 1      // pin as gpio out

#define	ZMP_GPIO_OUT_LOW			0
#define	ZMP_GPIO_OUT_HIGH		1

#define ZMP_GPIO_DIR1			0x00
#define ZMP_GPIO_OUT1			0x14
#define ZMP_GPIO_INPUT1         	0x28
#define ZMP_GPIO_INT_MASK1      	0x3c
#define ZMP_GPIO_INT_MODE1      	0x50
#define ZMP_GPIO_INTP1         	0x64
#define ZMP_GPIO_EDGE_STATUS1	0x78

#define ZMP_GPIO_DIR_BASE(pin)			(((pin)>>5)*4 + ZMP_GPIO_DIR1)
#define ZMP_GPIO_OUT_BASE(pin)			(((pin)>>5)*4 + ZMP_GPIO_OUT1)
#define ZMP_GPIO_IN_BASE(pin)			(((pin)>>5)*4 + ZMP_GPIO_INPUT1)
#define ZMP_GPIO_INTEN_BASE(pin)			(((pin)>>5)*4 + ZMP_GPIO_INT_MASK1)
#define ZMP_GPIO_INTM_BASE(pin)			(((pin)>>5)*4 + ZMP_GPIO_INT_MODE1)
#define ZMP_GPIO_INTPOL_BASE(pin)		(((pin)>>5)*4 + ZMP_GPIO_INTP1)
#define ZMP_GPIO_INTEDGE_BASE(pin)		(((pin)>>5)*4 + ZMP_GPIO_EDGE_STATUS1)

#define ZMP_GPIO_REG_SHIFT(pin)	((pin) % 32)

/*Analog Control Register 3 (0x080000A4)*/
#define ZMP_ANALOG_CTRL_REG3		(ZMP_VA_SYSCTRL + 0x0A4)

#define ZMP_SHAREPIN_CON0		(ZMP_VA_SYSCTRL + 0x178)
#define ZMP_SHAREPIN_CON1		(ZMP_VA_SYSCTRL + 0x17C)
#define ZMP_SHAREPIN_CON2		(ZMP_VA_SYSCTRL + 0x180)
#define ZMP_SHAREPIN_CON3		(ZMP_VA_SYSCTRL + 0x184)
#define ZMP_SHAREPIN_CON4		(ZMP_VA_SYSCTRL + 0x188)
#define ZMP_SHAREPIN_CON5		(ZMP_VA_SYSCTRL + 0x18C)

#define ZMP_SHAREPIN_CON6		(ZMP_VA_SYSCTRL + 0x190)

#define ZMP_GPIO_DRIVE_CON0		(ZMP_VA_SYSCTRL + 0x1D0)
#define ZMP_GPIO_DRIVE_CON1		(ZMP_VA_SYSCTRL + 0x1D4)
#define ZMP_GPIO_DRIVE_CON2		(ZMP_VA_SYSCTRL + 0x1D8)
#define ZMP_GPIO_DRIVE_CON3		(ZMP_VA_SYSCTRL + 0x1DC)
#define ZMP_GPIO_DRIVE_CON4		(ZMP_VA_SYSCTRL + 0x1E0)
#define ZMP_GPIO_DRIVE_CON5		(ZMP_VA_SYSCTRL + 0x1E4)

#define ZMP_PPU_PPD_EN0          (ZMP_VA_SYSCTRL + 0x194)
#define ZMP_PPU_PPD_EN1        	(ZMP_VA_SYSCTRL + 0x198)
#define ZMP_PPU_PPD_EN2        	(ZMP_VA_SYSCTRL + 0x19C)

#define ZMP_PPU_PPD_SEL0         (ZMP_VA_SYSCTRL + 0x1A0)
#define ZMP_PPU_PPD_SEL1         (ZMP_VA_SYSCTRL + 0x1A4)
#define ZMP_PPU_PPD_SEL2         (ZMP_VA_SYSCTRL + 0x1E8)

#define ZMP_PPU_PPD_EN_SEL       (ZMP_VA_SYSCTRL + 0x1F8)

#define ZMP_GPIO_IE_CON0			(ZMP_VA_SYSCTRL + 0x1EC)
#define ZMP_GPIO_IE_CON1			(ZMP_VA_SYSCTRL + 0x1F0)
#define ZMP_GPIO_IE_CON2			(ZMP_VA_SYSCTRL + 0x1F4)

#define ZMP_GPIO_SLEW_RATE		(ZMP_VA_SYSCTRL + 0x1FC)

#define ZMP_INTERRUPT_STATUS		(ZMP_VA_SYSCTRL + 0x30)
#define ZMP_ALWAYS_ON_PMU_CTRL0	(ZMP_VA_SYSCTRL + 0xDC)
#define ZMP_ALWAYS_ON_PMU_CTRL1	(ZMP_VA_SYSCTRL + 0xE0)

/* pins are just named gpio0..gpio85 */
#define ZMP110X_GPIO_PIN(a) PINCTRL_PIN(a, "gpio" #a)

/*
 *H3D-B changed drive map
 *00->10
 *01->11
 *10->00
 *11->01
 * */
#define DRIVE_STRENGTH(strength)	((strength) ^ 0x2)
//#define DRIVE_STRENGTH(strength)	(strength)

/*
 *	bit	8				7/6/5	4/3		2/1/0
 *		pupd_enable_bit	value	width	AO,I,I/O
 * */
#define GPIO_CLASS_IO	(0x0)
#define GPIO_CLASS_I	(0x1)
#define GPIO_CLASS_O	(0x2)
#define GPIO_CLASS_AO	(0x3)

#define SET_GPIO_CLASS(c)			((c) & 0x7)
#define SET_SHAREPIN_BITS_WIDTH(w)	(((w) & 0x3) << 3)
#define SET_VALUE_TO_GPIO(v)		(((v) & 0x7) << 5)
#define SET_PUPD_ENABLE_BIT(e)		(((e) & 0x1) << 8)

#define GET_GPIO_CLASS(c)		((c) & 0x7)
#define GET_SHAREPIN_WIDTH(c)	(((c) >> 3) & 0x3)
#define GET_GPIO_SET_VALUE(c)	(((c) >> 5) & 0x7)
#define GET_PUPD_ENABLE_BIT(c)	(((c) >> 8) & 0x1)

/*
 *GPIO_CFG - gpio
 *GPI_CFG -  GPI
 *GPO_CFG -  GPO
 *AO_CFG -   GPIO_AO
 * */
#define GPIO_CFG(width, value)	(SET_VALUE_TO_GPIO(value) |\
		SET_SHAREPIN_BITS_WIDTH(width) | SET_GPIO_CLASS(GPIO_CLASS_IO))
#define GPI_CFG(width, value)	(SET_VALUE_TO_GPIO(value) |\
		SET_SHAREPIN_BITS_WIDTH(width) | SET_GPIO_CLASS(GPIO_CLASS_I))
#define GPO_CFG(width, value)	(SET_VALUE_TO_GPIO(value) |\
		SET_SHAREPIN_BITS_WIDTH(width) | SET_GPIO_CLASS(GPIO_CLASS_O))
#define AO_CFG(width, value)	(SET_VALUE_TO_GPIO(value) |\
		SET_SHAREPIN_BITS_WIDTH(width) | SET_GPIO_CLASS(GPIO_CLASS_AO))

/*all capacity can't cfg*/
#define ALL_DIS			(0)
/*expect drive fixed, pull up fixed*/
#define DFIX_UP			(1)
/*expect drive fixed, pull down fixed*/
#define DFIX_DOWN		(2)
/*expect drive fixed, pull up fixed, IE enable fixed*/
#define DFIX_UP_IEEN	(3)
/*expect drive fixed, pull down fixed, IE enable fixed*/
#define DFIX_DOWN_IEEN	(4)
/*expect IE enable fixed*/
#define	IEEN			(5)
/*expect drive fixed, IE enable fixed*/
#define	DFIX_IEEN		(6)
/*all capacity support*/
#define ALL_EN			(0xf)

struct pin_pos {
	int offset;
	int width;
	int gpio0;
	int func0;
	int gpio1;
	int func1;
	int gpio2;
	int func2;
	int gpio3;
	int func3;
};

struct gpio_sharepin {
	int pin;
	void __iomem * sharepin_reg;
	void __iomem * drive_reg;
	u32 reg_off;

	void __iomem * pupd_en;
	void __iomem * pupd_sel;
	void __iomem * ie;
	u32 sel_bit;

	int slew_bit;
	int gpio_config;
	int capacity;
};

struct zmp_pinctrl {
	struct device *dev;
	struct clk *clk;
	void __iomem *gpio_base;
	void __iomem *mipi0_base;
	void __iomem *mipi1_base;
	int irq;

	unsigned int irq_type[ZMP110X_NUM_GPIOS];
	int fsel[ZMP110X_NUM_GPIOS];

	struct pinctrl_dev *pctl_dev;
	struct irq_domain *irq_domain;
	struct gpio_chip gc;
	struct pinctrl_gpio_range gpio_range;

	int GPI_MIPI0_pull_polarity;
	int GPI_MIPI1_pull_polarity;
	int GPI_MIPI0_pull_enable;
	int GPI_MIPI1_pull_enable;
	int mipi0_lanes_num;
	int mipi1_lanes_num;
	int ttl_io_voltage;

	spinlock_t lock;
};

static struct pinctrl_pin_desc zmp_gpio_pins[] = {
	ZMP110X_GPIO_PIN(0),
	ZMP110X_GPIO_PIN(1),
	ZMP110X_GPIO_PIN(2),
	ZMP110X_GPIO_PIN(3),
	ZMP110X_GPIO_PIN(4),
	ZMP110X_GPIO_PIN(5),
	ZMP110X_GPIO_PIN(6),
	ZMP110X_GPIO_PIN(7),
	ZMP110X_GPIO_PIN(8),
	ZMP110X_GPIO_PIN(9),
	ZMP110X_GPIO_PIN(10),
	ZMP110X_GPIO_PIN(11),
	ZMP110X_GPIO_PIN(12),
	ZMP110X_GPIO_PIN(13),
	ZMP110X_GPIO_PIN(14),
	ZMP110X_GPIO_PIN(15),
	ZMP110X_GPIO_PIN(16),
	ZMP110X_GPIO_PIN(17),
	ZMP110X_GPIO_PIN(18),
	ZMP110X_GPIO_PIN(19),
	ZMP110X_GPIO_PIN(20),
	ZMP110X_GPIO_PIN(21),
	ZMP110X_GPIO_PIN(22),
	ZMP110X_GPIO_PIN(23),
	ZMP110X_GPIO_PIN(24),
	ZMP110X_GPIO_PIN(25),
	ZMP110X_GPIO_PIN(26),
	ZMP110X_GPIO_PIN(27),
	ZMP110X_GPIO_PIN(28),
	ZMP110X_GPIO_PIN(29),
	ZMP110X_GPIO_PIN(30),
	ZMP110X_GPIO_PIN(31),
	ZMP110X_GPIO_PIN(32),
	ZMP110X_GPIO_PIN(33),
	ZMP110X_GPIO_PIN(34),
	ZMP110X_GPIO_PIN(35),
	ZMP110X_GPIO_PIN(36),
	ZMP110X_GPIO_PIN(37),
	ZMP110X_GPIO_PIN(38),
	ZMP110X_GPIO_PIN(39),
	ZMP110X_GPIO_PIN(40),
	ZMP110X_GPIO_PIN(41),
	ZMP110X_GPIO_PIN(42),
	ZMP110X_GPIO_PIN(43),
	ZMP110X_GPIO_PIN(44),
	ZMP110X_GPIO_PIN(45),
	ZMP110X_GPIO_PIN(46),
	ZMP110X_GPIO_PIN(47),
	ZMP110X_GPIO_PIN(48),
	ZMP110X_GPIO_PIN(49),
	ZMP110X_GPIO_PIN(50),
	ZMP110X_GPIO_PIN(51),
	ZMP110X_GPIO_PIN(52),
	ZMP110X_GPIO_PIN(53),
	ZMP110X_GPIO_PIN(54),
	ZMP110X_GPIO_PIN(55),
	ZMP110X_GPIO_PIN(56),
	ZMP110X_GPIO_PIN(57),
	ZMP110X_GPIO_PIN(58),
	ZMP110X_GPIO_PIN(59),
	ZMP110X_GPIO_PIN(60),
	ZMP110X_GPIO_PIN(61),
	ZMP110X_GPIO_PIN(62),
	ZMP110X_GPIO_PIN(63),
	ZMP110X_GPIO_PIN(64),
	ZMP110X_GPIO_PIN(65),
	ZMP110X_GPIO_PIN(66),
	ZMP110X_GPIO_PIN(67),
	ZMP110X_GPIO_PIN(68),
	ZMP110X_GPIO_PIN(69),
	ZMP110X_GPIO_PIN(70),
	ZMP110X_GPIO_PIN(71),
	ZMP110X_GPIO_PIN(72),
	ZMP110X_GPIO_PIN(73),
	ZMP110X_GPIO_PIN(74),
	ZMP110X_GPIO_PIN(75),
	ZMP110X_GPIO_PIN(76),
	ZMP110X_GPIO_PIN(77),
	ZMP110X_GPIO_PIN(78),
	ZMP110X_GPIO_PIN(79),
	ZMP110X_GPIO_PIN(80),
	ZMP110X_GPIO_PIN(81),
	ZMP110X_GPIO_PIN(82),
	ZMP110X_GPIO_PIN(83),
	ZMP110X_GPIO_PIN(84),
	ZMP110X_GPIO_PIN(85),
	ZMP110X_GPIO_PIN(86),
	ZMP110X_GPIO_PIN(87),
	ZMP110X_GPIO_PIN(88),
	ZMP110X_GPIO_PIN(89),
	ZMP110X_GPIO_PIN(90),
	ZMP110X_GPIO_PIN(91),
	ZMP110X_GPIO_PIN(92),
	ZMP110X_GPIO_PIN(93),
	ZMP110X_GPIO_PIN(94),
	ZMP110X_GPIO_PIN(95),
	ZMP110X_GPIO_PIN(96),
	ZMP110X_GPIO_PIN(97),
	ZMP110X_GPIO_PIN(98),
	ZMP110X_GPIO_PIN(99),
	ZMP110X_GPIO_PIN(100),
	ZMP110X_GPIO_PIN(101),
	ZMP110X_GPIO_PIN(102),
	ZMP110X_GPIO_PIN(103),
	ZMP110X_GPIO_PIN(104),
	ZMP110X_GPIO_PIN(105),
	ZMP110X_GPIO_PIN(106),
	ZMP110X_GPIO_PIN(107),
	ZMP110X_GPIO_PIN(108),
	ZMP110X_GPIO_PIN(109),
	ZMP110X_GPIO_PIN(110),
	ZMP110X_GPIO_PIN(111),
	ZMP110X_GPIO_PIN(112),
	ZMP110X_GPIO_PIN(113),
	ZMP110X_GPIO_PIN(114),
	ZMP110X_GPIO_PIN(115),
	ZMP110X_GPIO_PIN(116),
	ZMP110X_GPIO_PIN(117),
	ZMP110X_GPIO_PIN(118),
	ZMP110X_GPIO_PIN(119),
	ZMP110X_GPIO_PIN(120),
	ZMP110X_GPIO_PIN(121),
	ZMP110X_GPIO_PIN(122),
};

/* one pin per group */
static const char * const zmp_gpio_groups[] = {
	"gpio0",
	"gpio1",
	"gpio2",
	"gpio3",
	"gpio4",
	"gpio5",
	"gpio6",
	"gpio7",
	"gpio8",
	"gpio9",
	"gpio10",
	"gpio11",
	"gpio12",
	"gpio13",
	"gpio14",
	"gpio15",
	"gpio16",
	"gpio17",
	"gpio18",
	"gpio19",
	"gpio20",
	"gpio21",
	"gpio22",
	"gpio23",
	"gpio24",
	"gpio25",
	"gpio26",
	"gpio27",
	"gpio28",
	"gpio29",
	"gpio30",
	"gpio31",
	"gpio32",
	"gpio33",
	"gpio34",
	"gpio35",
	"gpio36",
	"gpio37",
	"gpio38",
	"gpio39",
	"gpio40",
	"gpio41",
	"gpio42",
	"gpio43",
	"gpio44",
	"gpio45",
	"gpio46",
	"gpio47",
	"gpio48",
	"gpio49",
	"gpio50",
	"gpio51",
	"gpio52",
	"gpio53",
	"gpio54",
	"gpio55",
	"gpio56",
	"gpio57",
	"gpio58",
	"gpio59",
	"gpio60",
	"gpio61",
	"gpio62",
	"gpio63",
	"gpio64",
	"gpio65",
	"gpio66",
	"gpio67",
	"gpio68",
	"gpio69",
	"gpio70",
	"gpio71",
	"gpio72",
	"gpio73",
	"gpio74",
	"gpio75",
	"gpio76",
	"gpio77",
	"gpio78",
	"gpio79",
	"gpio80",
	"gpio81",
	"gpio82",
	"gpio83",
	"gpio84",
	"gpio85",
	"gpio86",
	"gpio87",
	"gpio88",
	"gpio89",
	"gpio90",
	"gpio91",
	"gpio92",
	"gpio93",
	"gpio94",
	"gpio95",
	"gpio96",
	"gpio97",
	"gpio98",
	"gpio99",
	"gpio100",
	"gpio101",
	"gpio102",
	"gpio103",
	"gpio104",
	"gpio105",
	"gpio106",
	"gpio107",
	"gpio108",
	"gpio109",
	"gpio110",
	"gpio111",
	"gpio112",
	"gpio113",
	"gpio114",
	"gpio115",
	"gpio116",
	"gpio117",
	"gpio118",
	"gpio119",
	"gpio120",
	"gpio121",
	"gpio122",
};

static const char * const zmp_funcs[5] = {
	[0] = "default func",
	[1] = "func1",
	[2] = "func2",
	[3] = "func3",
	[4] = "func4",
};

//share pin func config in ZMP110X
static struct gpio_sharepin zmp_sharepin[] = {
	/* goio, uart3_rx */
	{0, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 8, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 20, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, uart0_rx, pwm4 */
	{1, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 0, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 0, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, uart0_tx, pwm3 */
	{2, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 2, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 1, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm0, mci2_d0 */
	{3, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 4, ZMP_PPU_PPD_EN0, 0, ZMP_GPIO_IE_CON0, 2, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, pwm1, uart3_tx, mci2_cmd */
	{4, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 6, ZMP_PPU_PPD_EN0, 0, ZMP_GPIO_IE_CON0, 3, -1, GPIO_CFG(2, 0B00), DFIX_DOWN},
	/* gpio, pwm2, uart3_rx, mci2_clk */
	{5, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 8, ZMP_PPU_PPD_EN0, 0, ZMP_GPIO_IE_CON0, 4, -1, GPIO_CFG(2, 0B00), DFIX_DOWN},
	/* gpio, uart1_rx, pwm0, i2c3_scl */
	{6, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 10, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 5, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm1, i2c3_sda */
	{7, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 12, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 6, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, uart1_cts, irda_rx */
	{8, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 14, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 7, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, uart1_rts */
	{9, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 16, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 8, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_mdc, i2c3_scl, pwm0 */
	{10, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 18, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 9, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_mdio, i2c3_sda */
	{11, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 20, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 10, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* spi0_cs, gpio */
	{12, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 28, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 30, -1, GPIO_CFG(2, 0B01), ALL_EN},
	/* gpio, rmii_txen */
	{13, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 22, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 11, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_txd0, mci2_d3 */
	{14, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 24, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 12, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_txd1,mci2_d2 */
	{15, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 26, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 13, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, gpo0_ao */
	{16, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpo1_ao */
	{17, 0, 0, 1, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, uart3_txd */
	{18, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 6, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 19, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_rxd0, mci2_clk */
	{19, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 28, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 14, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rmii_rxd1, mci2_cmd */
	{20, ZMP_SHAREPIN_CON0, ZMP_GPIO_DRIVE_CON0, 30, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 15, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, gpo2_ao */
	{21, 0, 0, 2, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpo3_ao */
	{22, 0, 0, 3, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, rmii_rxer, mci2_d1, pwm3 */
	{23, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 0, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 16, 28, GPIO_CFG(2, 0B00), DFIX_DOWN_IEEN},
	/* gpio, rmii_rxdv, mci2_d0, pwm4 */
	{24, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 2, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 17, 28, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, cis1_sclk */
	{25, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 6, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 19, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, cis_pclk, cis1_sclk */
	{26, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 8, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 20, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, cis_hsync, pwm3, i2c1_scl */
	{27, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 10, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 21, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, cis_vsync, pwm4, i2c1_sda */
	{28, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 12, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 22, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm0 */
	{29, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 10, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 21, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm1 */
	{30, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 12, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 22, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2c0_scl */
	{31, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 14, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 23, -1, GPIO_CFG(2, 0B00) | SET_PUPD_ENABLE_BIT(1), DFIX_UP},
	/* gpio, i2c0_sda */
	{32, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 16, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 24, -1, GPIO_CFG(2, 0B00) | SET_PUPD_ENABLE_BIT(1), DFIX_UP},
	/* gpio, mci0_cmd */
	{33, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 18, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 25, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_clk, uart2_tx */
	{34, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 20, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 26, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d0, uart2_rx, jtag_rstn */
	{35, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 22, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 27, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d1, uart1_rx, jtag_rtck */
	{36, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 24, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 28, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d2, pwm1, uart1_tx */
	{37, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 26, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 29, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d3, pwm1, jtag_tck */
	{38, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 28, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 30, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d4, pwm2, jtag_tms */
	{39, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 30, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 31, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d5, pwm3, jtag_tdi */
	{40, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 0, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 0, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d6, pwm4, jtag_tdout */
	{41, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 2, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 1, 29, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, mci0_d7 */
	{42, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 4, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 2, 29, GPIO_CFG(2, 0B00), ALL_EN},

	/* gpio, mci1_cmd, spi1_cs */
	{43, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 6, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 3, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, mci1_clk, spi1_sclk */
	{44, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 8, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 4, -1, GPIO_CFG(2, 0B00), DFIX_DOWN},
	/* gpio, mci1_d0, spi1_hold */
	{45, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 10, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 5, -1, GPIO_CFG(2, 0B00), DFIX_UP},

	
	/* gpio, mci1_d1, spi1_wp */
	{46, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 12, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 6, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, mci1_d2, spi1_dout */
	{47, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 14, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 7, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, mci1_d3, spi1_din */
	{48, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 16, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 8, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, opclk, mci2_d0, pwm1 */
	{49, ZMP_SHAREPIN_CON1, ZMP_GPIO_DRIVE_CON1, 4, ZMP_PPU_PPD_EN0, ZMP_PPU_PPD_SEL0, ZMP_GPIO_IE_CON0, 18, 28, GPIO_CFG(2, 0B00), IEEN},
	/* gpio, pwm0, uart1_tx */
	{50, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 18, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 9, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm1, uart1_cts */
	{51, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 20, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 10, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm2, uart1_rx */
	{52, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 22, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 11, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2s_dout, uart1_rts*/
	{53, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 24, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 12, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2s_mclk */
	{54, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 26, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 13, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2s_bclk */
	{55, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 28, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 14, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2s_lrclk */
	{56, ZMP_SHAREPIN_CON2, ZMP_GPIO_DRIVE_CON2, 30, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 15, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, i2s_din */
	{57, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 0, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 16, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm3, i2c2_sda */
	{58, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 2, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 17, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm4, i2c2_scl */
	{59, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 4, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 18, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, pwm0, ain1 */
	{60, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 14, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 23, -1, GPIO_CFG(2, 0B00), DFIX_DOWN_IEEN},
	/* gpio, pwm2 */
	{61, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 16, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 24, -1, GPIO_CFG(2, 0B00), DFIX_UP},
	/* gpio, pwm1 */
	{62, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 18, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 25, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio */
	{63, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 20, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 26, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio */
	{64, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 22, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 27, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio */
	{65, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 24, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 28, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio */
	{66, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 26, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 29, -1, GPIO_CFG(2, 0B00), ALL_EN},
	/* spi0_din, gpio */
	{67, ZMP_SHAREPIN_CON3, ZMP_GPIO_DRIVE_CON3, 30, ZMP_PPU_PPD_EN1, ZMP_PPU_PPD_SEL1, ZMP_GPIO_IE_CON1, 31, -1, GPIO_CFG(2, 0B01), DFIX_UP},
	/* spi0_dout, gpio */
	{68, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 0, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 0, -1, GPIO_CFG(2, 0B01), DFIX_UP},
	/* spi0_wp, gpio */
	{69, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 2, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 1, -1, GPIO_CFG(2, 0B01), DFIX_UP},
	/* spi0_hold, gpio */
	{70, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 4, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 2, -1, GPIO_CFG(2, 0B01), DFIX_UP},
	/* gpio, rgb_vogate/mpu_rd */
	{71, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 6, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 3, 0, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_vovsync/mpu_a0 */
	{72, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 8, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 4, 1, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_vohsync/mpu_cs */
	{73, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 10, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 5, 2, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_vopclk/mpu_wr,rgb_pclk */
	{74, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 12, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 6, 3, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d0/mpu_d0 */
	{75, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 14, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 7, 4, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d1/mpu_d1 */
	{76, ZMP_SHAREPIN_CON4, 0/*Note:b[17:18] drive_gpio76_92_cfg*/, 16, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 8, 5, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d2/mpu_d2 */
	{77, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 18, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 9, 6, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d3/mpu_d3 */
	{78, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 20, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 10, 7, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d4/mpu_d4 */
	{79, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 22, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 11, 8, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d5/mpu_d5 */
	{80, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 24, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 12, 9, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d6/mpu_d6 */
	{81, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 26, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 13, 10, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d7/mpu_d7 */
	{82, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 28, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 14, 11, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d8/mpu_d8, i2c2_scl */
	{83, ZMP_SHAREPIN_CON4, ZMP_GPIO_DRIVE_CON4, 30, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 15, 12, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d9/mpu_d9, i2c2_sda */
	{84, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 0, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 16, 13, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d10/mpu_d10, i2s_din */
	{85, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 2, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 17, 14, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d11/mpu_d11, i2s_lrclk */
	{86, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 4, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 18, 15, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d12/mpu_d12, i2s_bclk */
	{87, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 6, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 19, 16, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d13/mpu_d13, i2s_mclk */
	{88, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 8, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 20, 17, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d14/mpu_d14, i2s_dout */
	{89, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 10, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 21, 18, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d15/mpu_d15, pwm5 */
	{90, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 12, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 22, 19, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d16/mpu_d16, mci2_d2 */
	{91, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 14, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 23, 20, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d17/mpu_d17, mci2_d3 */
	{92, ZMP_SHAREPIN_CON5, 0/*Note:b[17]&[6] drive_gpio76_92_cfg*/, 16, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 24, 21, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d18, spi1_cs, mci2_cmd */
	{93, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 18, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 25, 22, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d19, spi1_sclk, mci2_clk */
	{94, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 20, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 26, 23, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d20, spi1_dout, mci2_d0 */
	{95, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 22, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 27, 24, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d21, spi1_din, mci2_d1 */
	{96, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 24, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 28, 25, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d22, spi1_hold, i2c2_sda */
	{97, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 26, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 29, 26, GPIO_CFG(2, 0B00), ALL_EN},
	/* gpio, rgb_d23, spi1_wp, i2c2_scl, spi1_din */
	{98, ZMP_SHAREPIN_CON5, ZMP_GPIO_DRIVE_CON5, 28, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 30, 27, GPIO_CFG(3, 0B000), ALL_EN},
	/* gpio */
	{99, 0, ZMP_GPIO_DRIVE_CON5, 30, ZMP_PPU_PPD_EN2, ZMP_PPU_PPD_SEL2, ZMP_GPIO_IE_CON2, 31, -1, GPIO_CFG(0, 0B0), ALL_EN},
	/* gpio */
	{100, 0, ZMP_PPU_PPD_EN_SEL, 0, ZMP_PPU_PPD_EN_SEL, 0/*Note: ie_reg3_gpio100_103_cfg*/, 0/*Note: ie_reg3_gpio100_103_cfg*/, 10, -1, GPIO_CFG(2, 0B0), ALL_EN},
	/* gpio */
	{101, 0, ZMP_PPU_PPD_EN_SEL, 2, ZMP_PPU_PPD_EN_SEL, 0/*Note: ie_reg3_gpio100_103_cfg*/, 0/*Note: ie_reg3_gpio100_103_cfg*/, 11, -1, GPIO_CFG(2, 0B0), ALL_EN},
	/* gpio */
	{102, 0, ZMP_PPU_PPD_EN_SEL, 4, ZMP_PPU_PPD_EN_SEL, 0/*Note: ie_reg3_gpio100_103_cfg*/, 0/*Note: ie_reg3_gpio100_103_cfg*/, 12, -1, GPIO_CFG(2, 0B0), ALL_EN},
	/* gpio */
	{103, 0, ZMP_PPU_PPD_EN_SEL, 6, ZMP_PPU_PPD_EN_SEL, 0/*Note: ie_reg3_gpio100_103_cfg*/, 0/*Note: ie_reg3_gpio100_103_cfg*/, 13, -1, GPIO_CFG(2, 0B0), ALL_EN},
	/* gpio, gpi4_ao */
	{104, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpi3_ao */
	{105, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpi2_ao */
	{106, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpi1_ao */
	{107, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpio, gpi0_ao */
	{108, 0, 0, 0, 0, 0, 0, 0, -1, AO_CFG(0, 0B0), ALL_DIS},
	/* gpi0 */
	{109, ZMP_ANALOG_CTRL_REG3, 0, 25, 0, 0, 0, 0, -1, GPI_CFG(1, 0B1), ALL_DIS},
	/* gpi1 */
	{110, ZMP_ANALOG_CTRL_REG3, 0, 27, 0, 0, 0, 0, -1, GPI_CFG(1, 0B1), ALL_DIS},
	/* gpi2 */
	{111, ZMP_SHAREPIN_CON6, 0, 30, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi3 */
	{112, ZMP_SHAREPIN_CON6, 0, 30, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi4 */
	{113, ZMP_SHAREPIN_CON6, 0, 29, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi5 */
	{114, ZMP_SHAREPIN_CON6, 0, 29, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi6 */
	{115, ZMP_SHAREPIN_CON6, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi7 */
	{116, ZMP_SHAREPIN_CON6, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi8 */
	{117, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi9 */
	{118, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi10 */
	{119, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi11 */
	{120, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi12 */
	{121, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
	/* gpi13 */
	{122, ZMP_SHAREPIN_CON5, 0, 31, 0, 0, 0, 0, -1, GPI_CFG(1, 0B0), DFIX_IEEN},
}; 

static struct pin_pos zmp_pin_pos[] = {
	/*offset,width,	gpio0,	func0,	gpio1,	func1,	gpio2,	func2	gpio3,	func3*/
	{22,	2,		58,		2,		97,		3,		84,		2,		-1,		-1},	/*TWI2_SDA*/
	{21,	1,		85,		2,		57,		1,		-1,		-1,		-1,		-1},	/*I2S_DIN*/
	{20,	1,		86,		2,		56,		1,		-1,		-1,		-1,		-1},	/*I2C_LRCLK*/
	{19,	1,		87,		2,		55,		1,		-1,		-1,		-1,		-1},	/*I2S_BCLK*/
	{18,	1,		8,		1,		51,		2,		-1,		-1,		-1,		-1},	/*UART1_CTS*/
	{16,	2,		6,		1,		36,		2,		52,		2,		-1,		-1},	/*UART1_RXD*/
	{15,	1,		5,		2,		0,		1,		-1,		-1,		-1,		-1},	/*UART3_RXD*/
	{14,	1,		7,		3,		11,		2,		-1,		-1,		-1,		-1},	/*TWI3_SDA*/
	{12,	2,		48,		2,		96,		2,		98,		4,		-1,		-1},	/*SPI1_DIN*/
	{11,	1,		47,		2,		95,		2,		-1,		-1,		-1,		-1},	/*SPI1_DOUT*/
	{10,	1,		46,		2,		98,		2,		-1,		-1,		-1,		-1},	/*SPI1_WP*/
	{9,		1,		45,		2,		97,		2,		-1,		-1,		-1,		-1},	/*SPI1_HOLD*/
	{8,		1,		44,		2,		94,		2,		-1,		-1,		-1,		-1},	/*SPI1_SCLK*/
	{7,		1,		43,		2,		93,		2,		-1,		-1,		-1,		-1},	/*SPI1_CS*/
	{6,		1,		14,		2,		92,		2,		-1,		-1,		-1,		-1},	/*MCI2_D[3]*/
	{5,		1,		15,		2,		91,		2,		-1,		-1,		-1,		-1},	/*MCI2_D[2]*/
	{4,		1,		23,		2,		96,		3,		-1,		-1,		-1,		-1},	/*MCI2_D[1]*/
	{2,		2,		3,		2,		24,		2,		49,		2,		95,		3},		/*MCI2_D[0]*/
	{0,		2,		4,		3,		20,		2,		93,		3,		-1,		-1},	/*MCI2_CMD*/
};

static const char * const irq_type_names[] = {
	[IRQ_TYPE_NONE] = "none",
	[IRQ_TYPE_EDGE_RISING] = "edge-rising",
	[IRQ_TYPE_EDGE_FALLING] = "edge-falling",
	[IRQ_TYPE_LEVEL_HIGH] = "level-high",
	[IRQ_TYPE_LEVEL_LOW] = "level-low",
};

static int pin_check_capacity_input_enable(int pin, int enable)
{
	if (!enable) {
		switch (zmp_sharepin[pin].capacity) {
			case ALL_DIS:
			case DFIX_UP_IEEN:
			case DFIX_DOWN_IEEN:
			case IEEN:
			case DFIX_IEEN:
				//pr_err("%s can't set pin:%d ie disable\n", __func__, pin);
				break;

			default:
				break;
		}
	}
	return 0;
}

static int pin_check_capacity_drive(int pin, int strength)
{
	switch (zmp_sharepin[pin].capacity) {
		case ALL_DIS:
		case DFIX_UP:
		case DFIX_DOWN:
		case DFIX_UP_IEEN:
		case DFIX_DOWN_IEEN:
		case DFIX_IEEN:
			//pr_err("%s can't set pin:%d drive, drive fixed\n", __func__, pin);
			break;

		default:
			break;
	}
	return 0;
}

static int pin_check_capacity_pull_polarity(int pin, int pullup)
{
	if (pullup) {
		switch (zmp_sharepin[pin].capacity) {
			case ALL_DIS:
			case DFIX_DOWN:
			case DFIX_DOWN_IEEN:
				//pr_err("%s can't set pin:%d pull-up polarity, fixed\n", __func__, pin);
				break;

			default:
				break;
		}
	} else {
		switch (zmp_sharepin[pin].capacity) {
			case ALL_DIS:
			case DFIX_UP:
			case DFIX_UP_IEEN:
				//pr_err("%s can't set pin:%d pull-down polarity, fixed\n", __func__, pin);
				break;

			default:
				break;
		}
	}
	return 0;
}

static int pin_check_capacity_pull_enable(int pin, int enable)
{
	if (enable) {
		switch (zmp_sharepin[pin].capacity) {
			case ALL_DIS:
				//pr_err("%s can't set pin:%d pull enable\n", __func__, pin);
				break;

			default:
				break;
		}
	} else {
		switch (zmp_sharepin[pin].capacity) {
			case ALL_DIS:
				//pr_err("%s can't set pin:%d pull disable\n", __func__, pin);
				break;

			default:
				break;
		}
	}

	return 0;
}

static int pin_check_capacity_slew_rate(int pin, int fast)
{
	if (zmp_sharepin[pin].slew_bit < 0)
		;//pr_err("%s can't set pin:%d slew rate\n", __func__, pin);

	return 0;
}

static int pin_check_capacity_direction(int pin, int input)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);

	if (input) {
		switch (gpio_class) {
			case GPIO_CLASS_O:
			case GPIO_CLASS_AO:
				//pr_err("%s can't set pin:%d as input\n", __func__, pin);
				break;

			default:
				break;
		}
	} else {
		switch (gpio_class) {
			case GPIO_CLASS_I:
			case GPIO_CLASS_AO:
				//pr_err("%s can't set pin:%d as output\n", __func__, pin);
				break;

			default:
				break;
		}
	}

	return 0;
}

static int gpi_and_mipi_pins_set_share_ex(struct zmp_pinctrl *pc, int pin, int sel)
{
	u32 regval;
	int mipi_lanes_num;
	void __iomem *mipi_base;

	switch (pin) {
		case 23:
			if (sel == 4) {
				/*here set AIN1 mux to pin GPIO23*/
				regval = __raw_readl(ZMP_ANALOG_CTRL_REG3);
				regval |= 1<<29;
				__raw_writel(regval, ZMP_ANALOG_CTRL_REG3);
			} else {
				regval = __raw_readl(ZMP_ANALOG_CTRL_REG3);
				regval &= ~(1<<29);
				__raw_writel(regval, ZMP_ANALOG_CTRL_REG3);
			}
			break;

		case 60:
			if (sel == 4) {
				/*here set AIN1 mux to pin GPIO60*/
				regval = __raw_readl(ZMP_ANALOG_CTRL_REG3);
				regval |= 1<<28;
				__raw_writel(regval, ZMP_ANALOG_CTRL_REG3);
			} else {
				regval = __raw_readl(ZMP_ANALOG_CTRL_REG3);
				regval &= ~(1<<28);
				__raw_writel(regval, ZMP_ANALOG_CTRL_REG3);
			}
			break;

		case 111:
		case 112:
		case 113:
		case 114:
		case 115:
		case 116:
		case 117:
		case 118:
		case 119:
		case 120:
		case 121:
		case 122:
			/*here set mipi-gpi pins extern paramters*/
			if (pin >= 111 && pin <= 116) {
				mipi_base = pc->mipi0_base;
				mipi_lanes_num = pc->mipi0_lanes_num;
			} else {
				mipi_base = pc->mipi1_base;
				mipi_lanes_num = pc->mipi1_lanes_num;
			}

			if (sel == 1) {
				/*dvp mode*/
				__raw_writeb(0x7d, mipi_base);
				__raw_writeb(0x3f, mipi_base + 0x20);
				__raw_writeb(0x01, mipi_base + 0xb3);
				__raw_writeb(pc->ttl_io_voltage & 0xff, mipi_base + 0xb8);
			} else if (sel == 2) {
				/*mipi mode*/
				__raw_writeb(0x7d, mipi_base);
				if (mipi_lanes_num == 1)
					__raw_writeb(0xf8, mipi_base + 0xe0);
				else if (mipi_lanes_num == 2) {
					/*2lanes mode is default, but I don't know default value*/
					__raw_writeb(0xf9, mipi_base + 0xe0);
				} else {
					pr_err("%s err mipi_lanes_num:%d, pin:%d\n", __func__, mipi_lanes_num, pin);
					return -1;
				}
			} else if (sel == 0) {
				/*GPI mode*/
				__raw_writeb(0x7d, mipi_base);
				__raw_writeb(0x3f, mipi_base + 0x20);
				__raw_writeb(0x01, mipi_base + 0xb3);
				
				__raw_writeb(0xb0, mipi_base + 0x0d);//disable pullup
				__raw_writeb(0x08, mipi_base + 0xb8);//2.5V ttl input enable
			} else {
				pr_err("%s don't support sel:%d, pin:%d\n", __func__, sel, pin);
				return -1;
			}
			break;

		case 109:
		case 110:
			/*had cfg in zmp_sharepin*/
			break;

		default:
			return -1;
			break;
	}

	return 0;
}

static int drive_gpio76_92_cfg(int gpio, int drive)
{
	u32 regval;

	switch (gpio) {
		case 76:
			/*ZMP_GPIO_DRIVE_CON4 Note:b[17:18]*/
			regval =__raw_readl(ZMP_GPIO_DRIVE_CON4);
			regval &= ~(0x3<<17);
			regval |= ((drive >> 1) & 0x1) << 17;
			regval |= (drive & 0x1) << 18;
			__raw_writel(regval, ZMP_GPIO_DRIVE_CON4);

			break;

		case 92:
			/*ZMP_GPIO_DRIVE_CON5 Note:b[17]&[6]*/
			regval =__raw_readl(ZMP_GPIO_DRIVE_CON5);
			regval &= ~(0x1<<17);
			regval &= ~(0x1<<6);
			regval |= ((drive >> 1) & 0x1) << 17;
			regval |= (drive & 0x1) << 6;
			__raw_writel(regval, ZMP_GPIO_DRIVE_CON5);
			break;

		default:
			return -1;
			break;
	}

	return 0;
}

static int ie_reg3_gpio100_103_cfg(int gpio, int ie_enable)
{
	int offset = 15;
	u32 regval;

	if ((gpio >= 100) && (gpio <= 103))
		offset += gpio - 100;
	else
		return -1;

	regval =__raw_readl(ZMP_PPU_PPD_EN_SEL);
	if (ie_enable)
		regval |= 1 << offset;
	else
		regval &= ~(1 << offset);
	__raw_writel(regval, ZMP_PPU_PPD_EN_SEL);

	return 0;
}

static int pupd_sel_reg3_gpio100_103_cfg(int gpio, int pu_enable)
{
	int offset = 20;
	u32 regval;

	if ((gpio >= 100) && (gpio <= 103))
		offset += gpio - 100;
	else
		return -1;

	regval =__raw_readl(ZMP_PPU_PPD_EN_SEL);
	if (pu_enable)
		regval &= ~(1 << offset);
	else
		regval |= 1 << offset;
	__raw_writel(regval, ZMP_PPU_PPD_EN_SEL);

	return 0;
}

static int pin_pos_cfg(int gpio, int fsel)
{
	int i;
	int num;
	int bits = 0;
	u32 regval;

	num = sizeof(zmp_pin_pos) / sizeof(zmp_pin_pos[0]);
	for (i = 0; i < num; i++) {
		if (gpio == zmp_pin_pos[i].gpio0 && fsel == zmp_pin_pos[i].func0) {
			bits = 0;
			break;
		}

		if (gpio == zmp_pin_pos[i].gpio1 && fsel == zmp_pin_pos[i].func1) {
			bits = 0x1;
			break;
		}

		if (gpio == zmp_pin_pos[i].gpio2 && fsel == zmp_pin_pos[i].func2) {
			bits = 0x2;
			break;
		}

		if (gpio == zmp_pin_pos[i].gpio3 && fsel == zmp_pin_pos[i].func3) {
			bits = 0x3;
			break;
		}
	}

	if (i < num) {
		regval = __raw_readl(ZMP_SHAREPIN_CON6);
		regval &= ~((~((~(unsigned long)0) << zmp_pin_pos[i].width)) << zmp_pin_pos[i].offset);
		regval |= bits << zmp_pin_pos[i].offset;
		__raw_writel(regval, ZMP_SHAREPIN_CON6);

		return 0;
	}

	return -1;
}

static void pmu_reg_write(u32 reg, u32 value)
{
	u32 regval;
	
	/*wait bit 17 pmu ready int status to be 1*/
	do {
	} while (!(__raw_readl(ZMP_INTERRUPT_STATUS) & (1<<17)));

	regval = (0x1<<21)|(0x2<<18)|(0x0<<17)
		|((reg&0x7)<<14)|(value&0x3FFF);
	__raw_writel(regval, ZMP_ALWAYS_ON_PMU_CTRL0);

	/*wait bit 17 pmu ready int status to be 1*/
	do {
	} while (!(__raw_readl(ZMP_INTERRUPT_STATUS) & (1<<17)));
}

static u32 pmu_reg_read(u32 reg)
{
	u32 regval;

	/*wait bit 17 pmu ready int status to be 1*/
	do {
	} while (!(__raw_readl(ZMP_INTERRUPT_STATUS) & (1<<17)));

	regval = (0x1<<21)|(0x2<<18)|(0x1<<17)
		|((reg&0x7)<<14);
	__raw_writel(regval, ZMP_ALWAYS_ON_PMU_CTRL0);

	/*wait bit 17 pmu ready int status to be 1*/
	do {
	} while (!(__raw_readl(ZMP_INTERRUPT_STATUS) & (1<<17)));

	return (__raw_readl(ZMP_ALWAYS_ON_PMU_CTRL1))&0x3FFF;
}

static int pin_set_share(struct zmp_pinctrl *pc, int pin, int sel)
{
	int sharepin_bits = GET_SHAREPIN_WIDTH(zmp_sharepin[pin].gpio_config);
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		u8 bit;
		if (pin == 16)
			bit = 0;
		else if (pin == 17)
			bit = 1;
		else if (pin == 21)
			bit = 2;
		else if (pin == 22)
			bit = 3;
		else if (pin == 104)
			bit = 4;
		else if (pin == 105)
			bit = 5;
		else if (pin == 106)
			bit = 6;
		else if (pin == 107)
			bit = 7;
		else if (pin == 108)
			bit = 8;
		else {
			pr_err("%s error pin:%d, sel:%d\n", __func__, pin, sel);
			return -1;
		}

		if (sel == 0) {
			regval = pmu_reg_read(0x4);
			regval |= 0x1;
			pmu_reg_write(0x4, regval);
			/* just config reg0_1 for function select */
			/* keep the default value of reg0_0 */

			regval = pmu_reg_read(0x0);
			regval &= ~(0x1<<bit);
			pmu_reg_write(0x0, regval);
		}else if (sel == 1) {
			regval = pmu_reg_read(0x4);
			regval |= 0x1;
			pmu_reg_write(0x4, regval);
			/* just config reg0_1 for function select */
			/* keep the default value of reg0_0 */

			regval = pmu_reg_read(0x0);
			regval |= (0x1<<bit);
			pmu_reg_write(0x0, regval);
		}
	} else {
		if (zmp_sharepin[pin].sharepin_reg != 0 &&
				sel <= (~(0xffffffff << sharepin_bits))) {
			regval = __raw_readl(zmp_sharepin[pin].sharepin_reg);
			regval &= ~((~(0xffffffff << sharepin_bits)) << zmp_sharepin[pin].reg_off);
			regval |= (sel << zmp_sharepin[pin].reg_off);
			__raw_writel(regval, zmp_sharepin[pin].sharepin_reg);
		}

		gpi_and_mipi_pins_set_share_ex(pc, pin, sel);
		pin_pos_cfg(pin, sel);
	}

	pc->fsel[pin] = sel;
	return 0;
}

static int pin_set_input_enable(int pin, int enable)
{
	int done = 0;
	int ie = enable ? 1:0;
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	pin_check_capacity_input_enable(pin, enable);

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		done = 1;
	} else {
		if (zmp_sharepin[pin].ie) {
			regval = __raw_readl(zmp_sharepin[pin].ie);
			regval &= ~(0x1<<zmp_sharepin[pin].sel_bit);
			regval |= (ie<<zmp_sharepin[pin].sel_bit);
			__raw_writel(regval, zmp_sharepin[pin].ie);
			done = 1;
		}

		if (!ie_reg3_gpio100_103_cfg(pin, ie))
			done = 1;
	}

	if (!done)
		;//pr_err("%s can't set pin:%d input enable\n", __func__, pin);
	return 0;
}

static int pin_get_input_enable(int pin)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		return 0;
	} else {
		if (zmp_sharepin[pin].ie) {
			regval = __raw_readl(zmp_sharepin[pin].ie);
			regval &= (0x1<<zmp_sharepin[pin].sel_bit);
			return (regval ? 1:0);
		}
	}
	pr_err("%s can't get pin:%d input enable\n", __func__, pin);
	return 0;
}

static int pin_set_drive(int pin, int strength)
{
	int done = 0;
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	strength = DRIVE_STRENGTH(strength);

	pin_check_capacity_drive(pin, strength);

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		done = 1;
	} else {
		if (zmp_sharepin[pin].drive_reg) {
			regval = __raw_readl(zmp_sharepin[pin].drive_reg);
			regval &= ~(0x3<<zmp_sharepin[pin].reg_off);
			regval |= ((strength&0x3)<<zmp_sharepin[pin].reg_off);
			__raw_writel(regval, zmp_sharepin[pin].drive_reg);
			done = 1;
		}

		if (!drive_gpio76_92_cfg(pin, strength))
			done = 1;
	}

	if (!done)
		;//pr_err("%s can't set pin:%d drive\n", __func__, pin);
	return 0;
}

static int pin_get_drive(int pin)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		return 0;
	} else {
		if (zmp_sharepin[pin].drive_reg) {
			regval = __raw_readl(zmp_sharepin[pin].drive_reg);
			regval &= 0x3<<zmp_sharepin[pin].reg_off;
			regval >>= zmp_sharepin[pin].reg_off;
			return DRIVE_STRENGTH(regval);
		}
	}
	pr_err("%s can't get pin:%d drive\n", __func__, pin);
	return 0;
}

static int pin_set_pull_polarity(struct zmp_pinctrl *pc, int pin, int pullup)
{
	int done = 0;
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	pin_check_capacity_pull_polarity(pin, pullup);

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		done = 1;
	} else {
		if (zmp_sharepin[pin].pupd_sel) {
			if (!pullup) {
				regval = __raw_readl(zmp_sharepin[pin].pupd_sel);
				regval |= (1<<zmp_sharepin[pin].sel_bit);
				__raw_writel(regval, zmp_sharepin[pin].pupd_sel);
			} else {
				regval = __raw_readl(zmp_sharepin[pin].pupd_sel);
				regval &= ~(1<<zmp_sharepin[pin].sel_bit);
				__raw_writel(regval, zmp_sharepin[pin].pupd_sel);
			}
			done = 1;
		}

		if (!pupd_sel_reg3_gpio100_103_cfg(pin, pullup))
			done = 1;
	}

	if (!done)
		;//pr_err("%s can't set pin:%d pull polarity\n", __func__, pin);
	return 0;
}

static int pin_get_pull_polarity(struct zmp_pinctrl *pc, int pin)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		return 0;
	} else {
		if (zmp_sharepin[pin].pupd_sel) {
			regval = __raw_readl(zmp_sharepin[pin].pupd_sel);
			regval &= (1<<zmp_sharepin[pin].sel_bit);
			return (regval ? 0:1);
		}
	}

	pr_err("%s can't get pin:%d pull polarity\n", __func__, pin);
	return 0;
}

static int pin_set_open_drain(struct zmp_pinctrl *pc, int pin, int enable)
{
	int bit = -1;
	u32 regval;

	if (pin == 31)
		bit = 30;
	else if (pin == 32)
		bit = 31;

	if (bit >= 0) {
		pr_debug("%s pin:%d, enable:%d\n", __func__, pin, enable);
		regval = __raw_readl(ZMP_PPU_PPD_EN_SEL);
		if (enable)
			regval |= (1<<bit);
		else
			regval &= ~(1<<bit);
		__raw_writel(regval, ZMP_PPU_PPD_EN_SEL);
	}

	return 0;
}

static int pin_set_pull_enable(struct zmp_pinctrl *pc, int pin, int enable)
{
	int done = 0;
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	int pupd_enable_value = GET_PUPD_ENABLE_BIT(zmp_sharepin[pin].gpio_config);
	u32 regval;

	pin_check_capacity_pull_enable(pin, enable);

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		done = 1;
	} else {
		/* enable PU/PD function */
		if (zmp_sharepin[pin].pupd_en) {
			int set_value;
			if (enable) {
				set_value = pupd_enable_value;
			} else {
				set_value = !pupd_enable_value;
			}
			regval = __raw_readl(zmp_sharepin[pin].pupd_en);
			regval &= ~(1<<zmp_sharepin[pin].sel_bit);
			regval |= (set_value<<zmp_sharepin[pin].sel_bit);
			__raw_writel(regval, zmp_sharepin[pin].pupd_en);
			done = 1;
		}

		return 0;
	}

	if (!done)
		;//pr_err("%s can't set pin:%d pull enable\n", __func__, pin);
	return 0;
}

static int pin_get_pull_enable(struct zmp_pinctrl *pc, int pin)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
		return 0;
	} else {
		if (zmp_sharepin[pin].pupd_en) {
			regval = __raw_readl(zmp_sharepin[pin].pupd_en);
			regval &= (1<<zmp_sharepin[pin].sel_bit);
			return (regval ? 0:1);
		}
	}

	pr_err("%s can't get pin:%d pull enable\n", __func__, pin);
	return 0;
}

static int pin_set_slew_rate(int pin, int fast)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	pin_check_capacity_slew_rate(pin, fast);

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
	} else {
		if (zmp_sharepin[pin].slew_bit != -1) {
			regval = __raw_readl(ZMP_GPIO_SLEW_RATE);
			regval &= ~(0x1<<zmp_sharepin[pin].slew_bit);
			regval |= ((fast ? 1:0)<<zmp_sharepin[pin].slew_bit);
			__raw_writel(regval, ZMP_GPIO_SLEW_RATE);
		} else {
			//pr_err("%s can't set pin:%d slew\n", __func__, pin);
		}
	}

	return 0;
}

static int pin_get_slew_rate(int pin)
{
	int gpio_class = GET_GPIO_CLASS(zmp_sharepin[pin].gpio_config);
	u32 regval;

	if (gpio_class == GPIO_CLASS_AO) {
		/* Always on gpio, do some special config here */
		/* keep the default IE/PU config value of reg0_0 */
	} else {
		if (zmp_sharepin[pin].slew_bit != -1) {
			regval = __raw_readl(ZMP_GPIO_SLEW_RATE);
			regval &= (0x1<<zmp_sharepin[pin].slew_bit);
			return (regval ? 1:0);
		} else {
			pr_err("%s cann't get pin:%d slew\n", __func__, pin);
		}
	}
	return 0;
}

static int gpio_set_share(struct zmp_pinctrl *pc, unsigned offset)
{
	int sel;
	unsigned long flags;

	sel = GET_GPIO_SET_VALUE(zmp_sharepin[offset].gpio_config);
	local_irq_save(flags);
	pin_set_share(pc, offset, sel);
	local_irq_restore(flags);

	return 0;
}

static inline unsigned int zmp_pinctrl_fsel_get(
		struct zmp_pinctrl *pc, unsigned pin)
{
	int status = pc->fsel[pin];

	return status;
}

static inline void zmp_pinctrl_fsel_set(
		struct zmp_pinctrl *pc, unsigned pin,
		unsigned sel)
{
	unsigned long flags;
	
	local_irq_save(flags);
	pin_set_share(pc, pin, sel);
	local_irq_restore(flags);
}

static int zmp_gpio_set_drive(struct gpio_chip *chip, unsigned offset, int strength)
{
	pin_set_drive(offset, strength);
	return 0;
}

static int zmp_gpio_get_drive(struct gpio_chip *chip, unsigned offset)
{
	return pin_get_drive(offset);
}

static int zmp_gpio_set_pull_polarity(struct gpio_chip *chip, unsigned offset, int pullup)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	pin_set_pull_polarity(pc, offset, pullup);

	return 0;
}

static int zmp_gpio_get_pull_polarity(struct gpio_chip *chip, unsigned offset)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	return pin_get_pull_polarity(pc, offset);
}

static int zmp_gpio_set_pull_enable(struct gpio_chip *chip, unsigned offset, int enable)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	pin_set_pull_enable(pc, offset, enable);
	return 0;
}

static int zmp_gpio_get_pull_enable(struct gpio_chip *chip, unsigned offset)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	return pin_get_pull_enable(pc, offset);
}

static int zmp_gpio_set_input_enable(struct gpio_chip *chip, unsigned offset, int enable)
{
	pin_set_input_enable(offset, enable);
	return 0;
}

static int zmp_gpio_get_input_enable(struct gpio_chip *chip, unsigned offset)
{
	return pin_get_input_enable(offset);
}

static int zmp_gpio_set_slew_rate(struct gpio_chip *chip, unsigned offset, int fast)
{
	pin_set_slew_rate(offset, fast);
	return 0;
}

static int zmp_gpio_get_slew_rate(struct gpio_chip *chip, unsigned offset)
{
	return pin_get_slew_rate(offset);
}

static inline void zmp_pinctrl_fsel_reset(
		struct zmp_pinctrl *pc, unsigned pin,
		unsigned fsel)
{
	unsigned long flags;
	u32 regval;
	int cur = pc->fsel[pin];

	if (cur == -1)
		return;

	local_irq_save(flags);
	if (zmp_sharepin[pin].sharepin_reg) {
		regval = __raw_readl(zmp_sharepin[pin].sharepin_reg);
		regval &= ~(0x3 << zmp_sharepin[pin].reg_off);
		__raw_writel(regval, zmp_sharepin[pin].sharepin_reg);
	}
	local_irq_restore(flags);

	pc->fsel[pin] = -1;
}


static int zmp_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int zmp_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	void __iomem * reg = pc->gpio_base + ZMP_GPIO_IN_BASE(offset);
	unsigned int bit = ZMP_GPIO_REG_SHIFT(offset);

	if (offset <= 108) {
		return (__raw_readl(reg) & (1<<bit)) ? 1:0;
	} else {
		u32 val;

		val = __raw_readl(pc->gpio_base + 0x34) >> 13;

		return (val & (1 << (offset - 109))) ? 1:0;
	}
}

static void zmp_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);
	void __iomem *reg = pc->gpio_base + ZMP_GPIO_OUT_BASE(offset);
	unsigned int bit = ZMP_GPIO_REG_SHIFT(offset);
	unsigned long flags;
	u32 regval;

	local_irq_save(flags);
	if (ZMP_GPIO_OUT_LOW == value) {
		regval = __raw_readl(reg);
		regval &= ~(1<<bit);
		__raw_writel(regval, reg);
	} else if (ZMP_GPIO_OUT_HIGH == value) {
		regval = __raw_readl(reg);
		regval |= (1<<bit);
		__raw_writel(regval, reg);		
	}
	local_irq_restore(flags);
}

static int zmp_gpio_direction_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	zmp_gpio_set(chip, offset, value);
	return pinctrl_gpio_direction_output(chip->base + offset);
}

static int zmp_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct zmp_pinctrl *pc = dev_get_drvdata(chip->dev);

	return irq_linear_revmap(pc->irq_domain, offset);
}

static struct gpio_chip zmp_gpio_chip = {
	.label = MODULE_NAME,
	.owner = THIS_MODULE,
	.request = gpiochip_generic_request,
	.free = gpiochip_generic_free,
	.direction_input = zmp_gpio_direction_input,
	.direction_output = zmp_gpio_direction_output,
	.get = zmp_gpio_get,
	.set = zmp_gpio_set,
	.set_drive = zmp_gpio_set_drive,
	.get_drive = zmp_gpio_get_drive,
	.set_pull_polarity = zmp_gpio_set_pull_polarity,
	.get_pull_polarity = zmp_gpio_get_pull_polarity,
	.set_pull_enable = zmp_gpio_set_pull_enable,
	.get_pull_enable = zmp_gpio_get_pull_enable,
	.set_input_enable = zmp_gpio_set_input_enable,
	.get_input_enable = zmp_gpio_get_input_enable,
	.set_slew_rate = zmp_gpio_set_slew_rate,
	.get_slew_rate = zmp_gpio_get_slew_rate,
	.to_irq = zmp_gpio_to_irq,
	.base = 0,
	.ngpio = ZMP110X_NUM_GPIOS,
	.can_sleep = false,
};

static void zmp_pinctrl_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct zmp_pinctrl *pc = irq_desc_get_handler_data(desc);
	unsigned long irq_status;
	int i;

	for (i = 0; i < 4; i++) {
		int off = 0;
		
		chained_irq_enter(chip, desc);
        irq_status = __raw_readl(pc->gpio_base + ZMP_GPIO_EDGE_STATUS1 + i * 4);
		for_each_set_bit(off, &irq_status, 32) {
		    /* check all int status and handle the Edge/level interrupt */
		    int pin_irq = irq_find_mapping(pc->irq_domain, i * 32 + off);

            generic_handle_irq(pin_irq);

		}
		chained_irq_exit(chip, desc);
	}
}

static void zmp_gpio_irq_enable(struct irq_data *data)
{
	struct zmp_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned pin = irqd_to_hwirq(data);
	void __iomem *reg = pc->gpio_base + ZMP_GPIO_INTEN_BASE(pin);
	unsigned bit = ZMP_GPIO_REG_SHIFT(pin);
	unsigned long flags;

	u32 regval;

	spin_lock_irqsave(&pc->lock, flags);
	regval = __raw_readl(reg);
	regval |= (1 << bit);
	__raw_writel(regval, reg);
	spin_unlock_irqrestore(&pc->lock, flags);
}

static void zmp_gpio_irq_disable(struct irq_data *data)
{
	struct zmp_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned pin = irqd_to_hwirq(data);
	void __iomem *reg = pc->gpio_base + ZMP_GPIO_INTEN_BASE(pin);
	unsigned bit = ZMP_GPIO_REG_SHIFT(pin);
	unsigned long flags;

	u32 regval;

	spin_lock_irqsave(&pc->lock, flags);
	regval = __raw_readl(reg);
	regval &= ~(1 << bit);
	__raw_writel(regval, reg);
	spin_unlock_irqrestore(&pc->lock, flags);
}

/* slower path for reconfiguring IRQ type */
static int __zmp_gpio_irq_set_type_enabled(struct zmp_pinctrl *pc,
	unsigned offset, unsigned int type)
{
	u32 regval;
	void __iomem *reg_inten = pc->gpio_base + ZMP_GPIO_INTEN_BASE(offset);
	void __iomem *reg_intm  = pc->gpio_base + ZMP_GPIO_INTM_BASE(offset);
	void __iomem *reg_intp  = pc->gpio_base + ZMP_GPIO_INTPOL_BASE(offset);

	int bit = ZMP_GPIO_REG_SHIFT(offset);
 	
	switch (type) {
	case IRQ_TYPE_NONE:
		if (pc->irq_type[offset] != type) {
			regval = __raw_readl(reg_inten);
			regval &= ~(1<<bit);
			__raw_writel(regval, reg_inten);
			pc->irq_type[offset] = type;
		}
		break;

	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		if (pc->irq_type[offset] != type) {
			regval = __raw_readl(reg_intm);
			regval |= (1<<bit);
			__raw_writel(regval, reg_intm);

			regval = __raw_readl(reg_intp);
			if (IRQ_TYPE_EDGE_RISING == type)
				regval &= ~(1<<bit);
			else
				regval |= (1<<bit);
			__raw_writel(regval, reg_intp);			
			pc->irq_type[offset] = type;
		}
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		if (pc->irq_type[offset] != type) {
			regval = __raw_readl(reg_intm);
			regval &= ~(1<<bit);
			__raw_writel(regval, reg_intm);

			regval = __raw_readl(reg_intp);
			if (IRQ_TYPE_LEVEL_HIGH == type)
				regval &= ~(1<<bit);
			else
				regval |= (1<<bit);
			__raw_writel(regval, reg_intp);			
			pc->irq_type[offset] = type;
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int zmp_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct zmp_pinctrl *pc = irq_data_get_irq_chip_data(data);
	unsigned pin = irqd_to_hwirq(data);
	

	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pc->lock, flags);

    /* 
    *  First irq_set_type called by __setup_irq,
    *  and then irq_enable  called by __setup_irq.
    *  refer to kernel/irq/manage.c.  fixed by zhang zhipeng (2019/5/3)
    */
	ret = __zmp_gpio_irq_set_type_enabled(pc, pin, type);

	if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(data, handle_edge_irq);
	else
		irq_set_handler_locked(data, handle_level_irq);
	spin_unlock_irqrestore(&pc->lock, flags);

	return ret;
}


/* 
 *  irq_ack called by handle_edge_irq when using edge trigger mode. 
 *  Author:zhang zhipeng
 *  date: 2019-5-3
 */
static void zmp_gpio_irq_ack(struct irq_data *d)
{
	struct zmp_pinctrl *pc = irq_data_get_irq_chip_data(d);
	unsigned pin = irqd_to_hwirq(d);
	

	/* Clear the IRQ status*/
    __raw_readl(pc->gpio_base + ZMP_GPIO_INTEDGE_BASE(pin));
    //__raw_writel(0 << bit,pc->gpio_base + ZMP_GPIO_INTEDGE_BASE(pin));

}

static struct irq_chip zmp_gpio_irq_chip = {
	.name = "zmp_gpio_edge",
    .irq_ack	= zmp_gpio_irq_ack,
	.irq_enable = zmp_gpio_irq_enable,
	.irq_disable = zmp_gpio_irq_disable,
	.irq_set_type = zmp_gpio_irq_set_type,
	.irq_mask = zmp_gpio_irq_disable,
	.irq_mask_ack = zmp_gpio_irq_disable,
	.irq_unmask = zmp_gpio_irq_enable,
};

static int zmp_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(zmp_gpio_groups);
}

static const char *zmp_pctl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned selector)
{
	return zmp_gpio_groups[selector];
}

static int zmp_pctl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned selector,
		const unsigned **pins,
		unsigned *num_pins)
{
	*pins = &zmp_gpio_pins[selector].number;
	*num_pins = 1;

	return 0;
}

static void zmp_pctl_pin_dbg_show(struct pinctrl_dev *pctldev,
		struct seq_file *s,
		unsigned offset)
{
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int fsel = zmp_pinctrl_fsel_get(pc, offset);
	const char *fname = zmp_funcs[fsel];
	int irq = irq_find_mapping(pc->irq_domain, offset);

	seq_printf(s, "function %s; irq %d (%s)",
		fname, irq, irq_type_names[pc->irq_type[offset]]);
}

static void zmp_pctl_dt_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *maps, unsigned num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (maps[i].type == PIN_MAP_TYPE_CONFIGS_PIN)
			kfree(maps[i].data.configs.configs);

	kfree(maps);
}

static int zmp_pctl_dt_node_to_map_func(struct zmp_pinctrl *pc,
		struct device_node *np, u32 pin, u32 fnum,
		struct pinctrl_map **maps)
{
	struct pinctrl_map *map = *maps;

	if (fnum >= ARRAY_SIZE(zmp_funcs)) {
		dev_err(pc->dev, "%s: invalid zmp,function %d\n",
			of_node_full_name(np), fnum);
		return -EINVAL;
	}

	map->type = PIN_MAP_TYPE_MUX_GROUP;
	map->data.mux.group = zmp_gpio_groups[pin];
	map->data.mux.function = zmp_funcs[fnum];
	(*maps)++;

	return 0;
}

static int zmp_pctl_dt_node_to_map_pull(struct zmp_pinctrl *pc,
		struct device_node *np, u32 pin, u32 pull,
		struct pinctrl_map **maps)
{
	struct pinctrl_map *map = *maps;
	unsigned long *configs;

	configs = kzalloc(sizeof(*configs), GFP_KERNEL);
	if (!configs)
		return -ENOMEM;
	configs[0] = pull;

	map->type = PIN_MAP_TYPE_CONFIGS_PIN;
	map->data.configs.group_or_pin = zmp_gpio_pins[pin].name;
	map->data.configs.configs = configs;
	map->data.configs.num_configs = 1;
	(*maps)++;

	return 0;
}

static int zmp_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
		struct device_node *np,
		struct pinctrl_map **map, unsigned *num_maps)
{
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct property *pins, *funcs, *pulls;
	int num_pins, num_funcs, num_pulls, maps_per_pin;
	struct pinctrl_map *maps, *cur_map;
	int i, err;
	u32 pin, func, pull;

	pins = of_find_property(np, "zmp,pins", NULL);
	if (!pins) {
		dev_err(pc->dev, "%s: missing zmp,pins property\n",
				of_node_full_name(np));
		return -EINVAL;
	}

	funcs = of_find_property(np, "zmp,function", NULL);
	pulls = of_find_property(np, "zmp,pull", NULL);

	if (!funcs && !pulls) {
		dev_err(pc->dev,
			"%s: neither zmp,function nor zmp,pull specified\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	num_pins = pins->length / 4;
	num_funcs = funcs ? (funcs->length / 4) : 0;
	num_pulls = pulls ? (pulls->length / 4) : 0;

	if (num_funcs > 1 && num_funcs != num_pins) {
		dev_err(pc->dev,
			"%s: zmp,function must have 1 or %d entries\n",
			of_node_full_name(np), num_pins);
		return -EINVAL;
	}

	if (num_pulls > 1 && num_pulls != num_pins) {
		dev_err(pc->dev,
			"%s: zmp,pull must have 1 or %d entries\n",
			of_node_full_name(np), num_pins);
		return -EINVAL;
	}

	maps_per_pin = 0;
	if (num_funcs)
		maps_per_pin++;
	if (num_pulls)
		maps_per_pin++;
	cur_map = maps = kzalloc(num_pins * maps_per_pin * sizeof(*maps),
				GFP_KERNEL);
	if (!maps)
		return -ENOMEM;

	for (i = 0; i < num_pins; i++) {
		err = of_property_read_u32_index(np, "zmp,pins", i, &pin);
		if (err)
			goto out;
		if (pin >= ARRAY_SIZE(zmp_gpio_pins)) {
			dev_err(pc->dev, "%s: invalid zmp,pins value %d\n",
				of_node_full_name(np), pin);
			err = -EINVAL;
			goto out;
		}

		if (num_funcs) {
			err = of_property_read_u32_index(np, "zmp,function",
					(num_funcs > 1) ? i : 0, &func);
			if (err)
				goto out;
			err = zmp_pctl_dt_node_to_map_func(pc, np, pin,
							func, &cur_map);
			if (err)
				goto out;
		}
		if (num_pulls) {
			err = of_property_read_u32_index(np, "zmp,pull",
					(num_pulls > 1) ? i : 0, &pull);
			if (err)
				goto out;
			err = zmp_pctl_dt_node_to_map_pull(pc, np, pin,
							pull, &cur_map);
			if (err)
				goto out;
		}
	}

	*map = maps;
	*num_maps = num_pins * maps_per_pin;
	return 0;

out:
	kfree(maps);
	return err;
}

static const struct pinctrl_ops zmp_pctl_ops = {
	.get_groups_count = zmp_pctl_get_groups_count,
	.get_group_name = zmp_pctl_get_group_name,
	.get_group_pins = zmp_pctl_get_group_pins,
	.pin_dbg_show = zmp_pctl_pin_dbg_show,
	.dt_node_to_map = zmp_pctl_dt_node_to_map,
	.dt_free_map = zmp_pctl_dt_free_map,
};

static int zmp_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(zmp_funcs);
}

static const char *zmp_pmx_get_function_name(struct pinctrl_dev *pctldev,
		unsigned selector)
{
	return zmp_funcs[selector];
}

static int zmp_pmx_get_function_groups(struct pinctrl_dev *pctldev,
		unsigned selector,
		const char * const **groups,
		unsigned * const num_groups)
{
	*groups = zmp_gpio_groups;
	*num_groups = ARRAY_SIZE(zmp_gpio_groups);

	return 0;
}

static int zmp_pmx_set(struct pinctrl_dev *pctldev,
		unsigned func_selector,
		unsigned group_selector)
{
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	zmp_pinctrl_fsel_set(pc, group_selector, func_selector);
	return 0;
}

static void zmp_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned offset)
{
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	/* disable by setting sahrepin cfg to default value */
	zmp_pinctrl_fsel_reset(pc, offset, pc->fsel[offset]);
}

static int zmp_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned offset,
		bool input)
{
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	u32 regval;
	void __iomem *reg = pc->gpio_base + ZMP_GPIO_DIR_BASE(offset);
	unsigned int bit = ZMP_GPIO_REG_SHIFT(offset);
	int gpio_cfg = input ?
		ePIN_AS_GPIO_IN : ePIN_AS_GPIO_OUT;

	pin_check_capacity_direction(offset, input);

	local_irq_save(flags);

	if (ePIN_AS_GPIO_IN == gpio_cfg){
		regval = __raw_readl(reg);
		regval &= ~(1<<bit);
		__raw_writel(regval, reg);
	}else if (ePIN_AS_GPIO_OUT == gpio_cfg){
		regval = __raw_readl(reg);
		regval |= (1<<bit);
		__raw_writel(regval, reg);
	}

	gpio_set_share(pc, offset);
	if (ePIN_AS_GPIO_IN == gpio_cfg)
		pin_set_input_enable(offset, 1);

	local_irq_restore(flags);

	return 0;
}

static const struct pinmux_ops zmp_pmx_ops = {
	.get_functions_count = zmp_pmx_get_functions_count,
	.get_function_name = zmp_pmx_get_function_name,
	.get_function_groups = zmp_pmx_get_function_groups,
	.set_mux = zmp_pmx_set,
	.gpio_disable_free = zmp_pmx_gpio_disable_free,
	.gpio_set_direction = zmp_pmx_gpio_set_direction,
};

static int zmp_pinconf_get(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *config)
{
	/* No way to read back config in HW */

	return -ENOTSUPP;
}

static int zmp_pinconf_set(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *configs,
			unsigned num_configs)
{
	unsigned long flags;
	u8 pupd, drive, ie, slew;
	struct zmp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	if (num_configs != 1)
		return -EINVAL;

	/* for config value bit[31:24]--slew rate, bit[23:16]--ie, bit[15:8]--drive, bit[7:0]--pupd */
	pupd = (configs[0] & 0xFF);
	drive = ((configs[0]>>8) & 0xFF);
	ie = ((configs[0]>>16) & 0xFF);
	slew = ((configs[0]>>24) & 0xFF);

	local_irq_save(flags);
	pin_set_open_drain(pc, pin, (pupd & 0x80) ? 1:0);
	pin_set_pull_enable(pc, pin, (pupd & 0x10) ? 1:0);
	pin_set_pull_polarity(pc, pin, (pupd & 0x01) ? 0:1);
	pin_set_drive(pin, drive & 0x3);
	pin_set_input_enable(pin, ie & 0x1);
	pin_set_slew_rate(pin, slew & 0x1);
	local_irq_restore(flags);

	return 0;
}

/* for gpio PU/PD config */
static const struct pinconf_ops zmp_pinconf_ops = {
	.pin_config_get = zmp_pinconf_get,
	.pin_config_set = zmp_pinconf_set,
};

static struct pinctrl_desc zmp_pinctrl_desc = {
	.name = MODULE_NAME,
	.pins = zmp_gpio_pins,
	.npins = ARRAY_SIZE(zmp_gpio_pins),
	.pctlops = &zmp_pctl_ops,
	.pmxops = &zmp_pmx_ops,
	.confops = &zmp_pinconf_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_gpio_range zmp_pinctrl_gpio_range = {
	.name = MODULE_NAME,
	.npins = ZMP110X_NUM_GPIOS,
};

static int zmp_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct zmp_pinctrl *pc;
	struct resource iomem;
	int ret, i;

	BUILD_BUG_ON(ARRAY_SIZE(zmp_gpio_pins) != ZMP110X_NUM_GPIOS);

	dev_err(dev, "%s %d\n",__func__,__LINE__);

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	platform_set_drvdata(pdev, pc);
	pc->dev = dev;

	/* gpio registers */
	ret= of_address_to_resource(np, 0, &iomem);
	if (ret) {
		dev_err(dev, "could not get IO memory\n");
		return ret;
	}

	pc->gpio_base = devm_ioremap_resource(dev, &iomem);
	if (IS_ERR(pc->gpio_base))
		return PTR_ERR(pc->gpio_base);

	/* mipi0 registers */
	ret= of_address_to_resource(np, 1, &iomem);
	if (ret) {
		dev_err(dev, "could not get mipi0 memory\n");
		return ret;
	}

	pc->mipi0_base = devm_ioremap_resource(dev, &iomem);
	if (IS_ERR(pc->mipi0_base))
		return PTR_ERR(pc->mipi0_base);

	/* mipi1 registers */
	ret= of_address_to_resource(np, 2, &iomem);
	if (ret) {
		dev_err(dev, "could not get mipi1 memory\n");
		return ret;
	}

	pc->mipi1_base = devm_ioremap_resource(dev, &iomem);
	if (IS_ERR(pc->mipi1_base))
		return PTR_ERR(pc->mipi1_base);

	pc->gc = zmp_gpio_chip;
	pc->gc.dev = dev;
	pc->gc.of_node = np;

	pc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pc->clk)) {
		ret = PTR_ERR(pc->clk);
		return ret;
	}

	clk_prepare_enable(pc->clk);

	pc->irq = platform_get_irq(pdev, 0);
	if (pc->irq < 0) {
		ret = pc->irq;
		goto clk_err;
	}
	pr_err("zmp_pinctrl_probe irq: %d\n", pc->irq);
	
	pc->irq_domain = irq_domain_add_linear(np, ZMP110X_NUM_GPIOS,
			&irq_domain_simple_ops, pc);
	if (!pc->irq_domain) {
		dev_err(dev, "could not create IRQ domain\n");
		ret = -ENOMEM;
		goto clk_err;
	}

	for (i = 0; i < ZMP110X_NUM_GPIOS; i++) {
		int irq = irq_create_mapping(pc->irq_domain, i);
		//pr_debug("i = %d irq = %d\n", i, irq);
		pc->irq_type[i] = 0;
		pc->fsel[i] = -1;
		irq_set_chip_and_handler(irq, &zmp_gpio_irq_chip,
				handle_level_irq);
		irq_set_chip_data(irq, pc);
	}

	irq_set_chained_handler_and_data(pc->irq,
						 zmp_pinctrl_irq_handler, pc);

	ret = gpiochip_add(&pc->gc);
	if (ret) {
		dev_err(dev, "could not add GPIO chip\n");
		goto clk_err;
	}

	pc->pctl_dev = pinctrl_register(&zmp_pinctrl_desc, dev, pc);
	if (IS_ERR(pc->pctl_dev)) {
		gpiochip_remove(&pc->gc);
		ret = PTR_ERR(pc->pctl_dev);
		goto clk_err;
	}

	pc->gpio_range = zmp_pinctrl_gpio_range;
	pc->gpio_range.base = pc->gc.base;
	pc->gpio_range.gc = &pc->gc;
	pinctrl_add_gpio_range(pc->pctl_dev, &pc->gpio_range);

	return 0;

clk_err:
	clk_disable_unprepare(pc->clk);
	clk_put(pc->clk);

	return ret;
}

static int zmp_pinctrl_remove(struct platform_device *pdev)
{
	struct zmp_pinctrl *pc = platform_get_drvdata(pdev);

	pinctrl_unregister(pc->pctl_dev);
	gpiochip_remove(&pc->gc);
	clk_disable_unprepare(pc->clk);
	clk_put(pc->clk);

	return 0;
}

static const struct of_device_id zmp_pinctrl_match[] = {
	{ .compatible = "zlgmcu,zmp110x-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, zmp_pinctrl_match);

static struct platform_driver zmp_pinctrl_driver = {
	.probe = zmp_pinctrl_probe,
	.remove = zmp_pinctrl_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = zmp_pinctrl_match,
	},
};

static int __init zmp_pinctrl_init(void)
{
	return platform_driver_register(&zmp_pinctrl_driver);
}

static void __exit zmp_pinctrl_exit(void)
{
	platform_driver_unregister(&zmp_pinctrl_driver);
}

subsys_initcall(zmp_pinctrl_init);
module_exit(zmp_pinctrl_exit);

MODULE_DESCRIPTION("ZMP110x sharepin control driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.02");
