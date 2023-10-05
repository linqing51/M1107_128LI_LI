/*
 * ZMP1107 lcd driver
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ZMP1107_FB_H__
#define __ZMP1107_FB_H__

#include <linux/fb.h>
#include <asm/io.h>
#include <linux/ioctl.h>

#define true  1
#define false 0

#define MHz	(1000*1000)
#define MPU_INPUT_ON             (~(1<<29))
#define MPU_INPUT_OFF		     (~(0<<29))
#define MPU_READ_NOW_SET         (1<<28)
#define MPU_READ_NOW_CLEAR       (~(1<<28))
#define MPU_RD_SIG_ON            (~(1<<27))
#define MPU_CS_SIG_ON        	 (~(1<<26))
#define MPU_RS_CMD               (~(1<<24))

#define ZMP_LCD_MPU_CMD            (0 << 24)      /* LCD command */
#define ZMP_LCD_MPU_DATA           (1 << 24)      /* LCD data */

/*MPU's command and register */
#define MPU_WRITE_DATA_CMD  0x2C
#define MPU_DIRECTION_REG   0x36

#define LOGO_FILE_NAME					"LOGO"

/*portrait */
#define L2R_U2D  0 /*from left to right,from top to bottom. */
#define L2R_D2U  1 /*from left to right,from bottom to top. */
#define R2L_U2D  2 /*from right to left,from top to bottom. */
#define R2L_D2U  3 /*from right to left,from bottom to top. */
/*landscape */
#define U2D_L2R  4 /*from top to bottom, from left to right*/
#define U2D_R2L  5 /*from top to bottom, from right to left*/
#define D2U_L2R  6 /*from bottom to top, from left to right*/
#define D2U_R2L  7 /*from bottom to top, from right to left*/


//红色在高8位，蓝色在低8位，绿色在中间8位
enum lcd_24bit_rgb_color_val{
	LCD_24BIT_RGB_COLOR_BLACK = 0x00000000,				/* black*/
	LCD_24BIT_RGB_COLOR_RED = 0x00FF0000,				/* red */
	LCD_24BIT_RGB_COLOR_GREEN = 0x0000FF00,				/* green */
	LCD_24BIT_RGB_COLOR_BLUE = 0x000000FF, 				/* blue */
	LCD_24BIT_RGB_COLOR_YELLOW = 0x00FFFF00,			/* yellow */
	LCD_24BIT_RGB_COLOR_CYAN = 0x0000FFFF,				/* cyan*/
	LCD_24BIT_RGB_COLOR_FUCHSIN = 0x00FF00FF,			/*  fuchsin*/
	LCD_24BIT_RGB_COLOR_WHITE = 0x00FFFFFF, 			/* white */
};

enum lcd_mpu_dat_width {
    MPU8BITS  = 0b00,
    MPU16BITS  = 0b01,
    MPU18BITS = 0b10,
    MPU9BITS = 0b11,
};
enum lcd_mpu_color_mode {
    COLOR_MODE4K = 0b0,
    COLOR_MODE64K = 0b1,
};

/* bit[23:16] red, bit[15:8] blue, bit[7:0] green */
enum lcd_rgb_bg_color {
	LCD_RGB_COLOR_BLACK = 0x00000000,
	LCD_RGB_COLOR_RED = 0x00FF0000,
	LCD_RGB_COLOR_GREEN = 0x0000FF00,
	LCD_RGB_COLOR_BLUE = 0x000000FF,
	LCD_RGB_COLOR_YELLOW = 0x00FFFF00,
	LCD_RGB_COLOR_CYAN = 0x0000FFFF,
	LCD_RGB_COLOR_FUCHSIN = 0x00FF00FF,
	LCD_RGB_COLOR_WHITE = 0x00FFFFFF,
};

enum lcd_dummy_seq {
	LCD_DUMMY_BACK = 0,
	LCD_DUMMY_FRONT,
};

enum lcd_rgb_seq_sel {
	LCD_RGB_SEQ_CONSTANT = 0,
	LCD_RGB_SEQ_DIFFERENT,
};

/**
 * color format of the rgb channel
 * support RGB888, BGR888, RGB565, BGR565
 */
enum lcd_cfmt {
	FMT_ARGB   = 0b1101,
	FMT_ABGR   = 0b1100,
	FMT_RGBA   = 0b0101,
	FMT_BGRA   = 0b0100,
	FMT_RGB888 = 0b011,
	FMT_BGR888 = 0b010,
	FMT_RGB565 = 0b001,
	FMT_BGR565 = 0b000,
};

enum lcd_rgb_seq {
	LCD_SEQ_RGB = 0,
	LCD_SEQ_RBG,
	LCD_SEQ_GRB,
	LCD_SEQ_GBR,
	LCD_SEQ_BRG,
	LCD_SEQ_BGR,
};

//RGB LCD 8 bit 位宽所需要设置的显示模式
enum lcd_tpg_mode{
	LCD_RGB_TPG052 = 0,
	LCD_RGB_TPG051,
};

enum lcd_sig_pol {
	LCD_SIG_POS = 0,
	LCD_SIG_NEG,
};

enum lcd_rgb_bus_width {
	LCD_RGB_BUS_WIDTH_8BIT = 0,
	LCD_RGB_BUS_WIDTH_16BIT,
	LCD_RGB_BUS_WIDTH_18BIT,
	LCD_RGB_BUS_WIDTH_24BIT,
};

enum lcd_rgb_dma_mode {
	LCD_RGB_DMA_NORMAL_MODE = 0,
	LCD_RGB_DMA_2X_MODE,
};

//DPHY  DSI TXD
enum dsi_txd {
	LCD_DSI_TXD0 = 0,
	LCD_DSI_TXD1,
	LCD_DSI_TXD2,
	LCD_DSI_TXD3,
	LCD_DSI_TXD4,
};

//DPHY  DSI TXD
enum dsi_data_lane {
	LCD_MIPI_DAT0_LANE = 0,
	LCD_MIPI_DAT1_LANE,
	LCD_MIPI_DAT2_LANE,
	LCD_MIPI_DAT3_LANE,
	LCD_MIPI_CLK_LANE,
};

/**
 * zmp lcd controller output interface definition
 *   IF_MPU: output MPU lcd panel signal
 *   IF_RGB: output RGB lcd panel signal
 *   IF_DSI: output RGB lcd panel signal by DSI interface
 */
enum lcdc_if {
	DISP_IF_RGB = 0,
	DISP_IF_DSI,
	DISP_IF_MPU,
};

/**
 * initial mpu: data or command
 */

enum mpu_ctrl{
    LCD_MPU_CMD = 0,
    LCD_MPU_DATA,
};

struct lcdc_panel_info {
	const char *name;

	dma_addr_t paddr; /* physical address of the buffer */
	void * vaddr; /* virtual address of the buffer */
	u32 paddr_offset; /* fb paddr offset */
	u32 size; /* fb size */

	dma_addr_t paddr_shadow; /* physical address of the buffer */
	void * vaddr_shadow; /* virtual address of the buffer */

	u32 rgb_if;  //0 for RGB, 1 for MIPI, 2 for MPU
	u32 rgb_seq; //0 for RGB, 1 for BGR
	/* lcd display size */
	u32 width;
	u32 height;

	/* lcd timing info */
	u32 pclk_div;		/* pixel clock = Peri_pll/(pclk_div+1) */
	u32 thpw;
	u32 thb;
	u32 thd;
	u32 thf;
	u32 tvpw;
	u32 tvb;
	u32 tvd;
	u32 tvf;
    /*mpu write timing */
    u32 wr_cycle;
	u32 wr_low;
    u32 panel_direction;

	enum lcd_sig_pol pclk_pol;
	enum lcd_sig_pol hsync_pol;
	enum lcd_sig_pol vsync_pol;
	enum lcd_sig_pol vogate_pol;

	/* lcd data bus width */
	enum lcd_rgb_bus_width bus_width;
    /*mpu panel setting */
    enum lcd_mpu_dat_width   data_width;
    enum lcd_mpu_color_mode   color_mode;
    enum lcd_sig_pol       a0_pol;
    u32 dir;
    u32 wramcmd;
    u32 setxcmd;
    u32 setycmd;
};

struct lcdc_rgb_input_info {
	/* rgb input data size */
	u32 width;
	u32 height;
	u32 h_offset;
	u32 v_offset;
	u32 fmt0;  //rgb input data fmt
	u32 fmt1;  //rgb input data fmt
	u32 rgb_seq;  //rgb sequence
	u32 a_location; //for 32bits input data
	u32 bpp;
};

struct lcdc_rgb_info {
	u32 alarm_empty;
	u32 alarm_full;

	enum lcd_rgb_dma_mode dma_mode;

	//for 8bit rgb panel settings
	enum lcd_dummy_seq dummy_seq;
	enum lcd_rgb_seq_sel rgb_seq_sel;
	enum lcd_rgb_seq rgb_odd_seq;
	enum lcd_rgb_seq rgb_even_seq;
	enum lcd_tpg_mode tpg_mode;
	u32 blank_sel;
	u32 pos_blank_level;
	u32 neg_blank_level;

    //for vpage function settings
	u32 vpage_en;
	u32 vpage_hsize;
	u32 vpage_vsize;
	u32 vpage_hoffset;
	u32 vpage_voffset;

	//background color setting
	enum lcd_rgb_bg_color bg_color;

    // rgb timming setting
	u32 vh_delay_en;
	u32 vh_delay_cycle;
	u32 v_unit;

	// setting for lcd controller update sw config
	u32 alert_line;
};

struct lcdc_dsi_info {
    //mipi lcd lane config
	u32 num_lane;
	u32 txd0;
	u32 txd1;
	u32 txd2;
	u32 txd3;
	u32 txd4;

    //mipi dsi controller setting
	u32 noncontinuous_clk;
	u32 t_pre;
	u32 t_post;
	u32 tx_gap;
	u32 autoinsert_eotp;
	u32 htx_to_count;
	u32 lrx_to_count;
	u32 bta_to_count;
	u32 t_wakeup;
	u32 pix_payload_size;
	u32 pix_fifo_level;
	u32 if_color_coding;
	u32 pix_format;
	u32 vsync_pol;
	u32 hsync_pol;
	u32 video_mode;
	u32 hfp;
	u32 hbp;
	u32 hsa;
	u32 mult_pkts_en;
	u32 vbp;
	u32 vfp;
	u32 vsa;
	u32 bllp_mode;
	u32 use_null_pkt_bllp;
	u32 vactive;
	u32 vc;
	u32 cmd_type;
	u32 dsi_clk;
};

/**
 * the translucence value of osd channel
 * the left side is percent of translucence,
 * the right side is the value of the register
 */
enum zmp_lcd_osd_alpha {
	OSD_TRANS_100 = 0x0,
	OSD_TRANS_87  = 0x1,
	OSD_TRANS_75  = 0x2,
	OSD_TRANS_62  = 0x3,
	OSD_TRANS_50  = 0x4,
	OSD_TRANS_37  = 0x5,
	OSD_TRANS_25  = 0x6,
	OSD_TRANS_12  = 0x7,
	OSD_TRANS_0   = 0x8
};

/**
 * macros for set and get bit region of a register
 * NOTE: each macro defined below can be used, I prefer the latter
 */
#if 0

#define clear_low_bits(val, len)        (((len)==sizeof(val)*8)?0:((val) >> (len) << (len)))
#define mask(msb, lsb)                  clear_low_bits(~clear_low_bits(~0UL, (msb+1)), (lsb))

#else

#define clear_low_bits(val, len)        ((val) >> (len) << (len))
#define clear_high_bits(val, len)       ((val) << (len) >> (len))
#define mask(msb, lsb)                  clear_high_bits(clear_low_bits(~0UL, (lsb)), (sizeof(~0UL)*8 - 1 - (msb)))

#endif

#if 0

#define lcdc_set_reg(val, reg, msb, lsb) __raw_writel((__raw_readl(reg) & ~mask((msb), (lsb))) | (((val) & mask((msb)-(lsb), 0)) << (lsb)), \
						     (reg))

#else

#define lcdc_set_reg(val, reg, msb, lsb) __raw_writel((__raw_readl(reg) & ~mask((msb), (lsb))) | (((val) << (lsb)) & mask((msb), (lsb))), \
						     (reg))

#endif

#define lcdc_get_reg(reg, msb, lsb) ((__raw_readl(reg) & mask(msb, lsb)) >> lsb)

#define IRQ_DPI_HOST_ERR			(1 << 19)
#define IRQ_BUF_EMPTY				(1 << 18)
#define IRQ_ALERT_VALID				(1 << 17)
#define IRQ_UPDATE_DONE				(1 << 7)
#define IRQ_RGB_FRAME_START			(1 << 4)
#define IRQ_RGB_FRAME_END			(1 << 3)
#define IRQ_MPU_FRAME_END			(1 << 2)
#define IRQ_MPU_FRAME_START			(1 << 1)
#define IRQ_SYS_ERROR				(1)


/* convert r,g,b to a pack format */
#define ZMPRGB(r, g, b) ((((r)&0xff) << 16) | (((g)&0xff) << 8)| ((b)&0xff))

/**
 * private data struction of zmp lcd driver
 */
struct zmp_fb {
	struct platform_device *pdev;
	struct fb_info *fb;
	struct fb_info *fb_shadow;

	int irq;
	int reset_pin;
	struct clk *lcdc_clk;
	struct clk *dsi_clk;
	void __iomem *lcdc_reg;
	void __iomem *dsi_reg;

	struct delayed_work fb_irq_work;

	struct lcdc_rgb_input_info *rgb_input;
	struct lcdc_rgb_info *rgb_info;
	struct lcdc_dsi_info *dsi_info;
	struct lcdc_panel_info *panel_info;
	struct lcdc_osd_info *osd_info;

	u32 pseudo_palette[16]; //for fb console use
	u32 fb_type;  /* fb type: 0 single fb, 1 double fb */
	u32 fb_id;  /* current fb id */
	bool par_change; /* is parameter changed or not */
	bool lcd_go; /* is panel refreshing or not */
	int fb_adj_size;
	u32 lcd_ctrl_inited; /* for check uboot has display logo */

    wait_queue_head_t   wait;
    unsigned int        count;
};

#endif /* __ZMP1107_FB_H__ */
