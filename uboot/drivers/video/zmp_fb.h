/*
 * Copyright (C) 2012 Samsung Electronics
 *
 * Author: InKi Dae <inki.dae@samsung.com>
 * Author: Donghwa Lee <dh09.lee@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _ZMP_FB_H_
#define _ZMP_FB_H_


#define ZMP_PA_SYSCTRL				(0x08000000)
#define ZMP_CLK_GATE_CTRL0			(ZMP_PA_SYSCTRL + 0x001C)
#define ZMP_SW_RESET_CTRL0			(ZMP_PA_SYSCTRL + 0x0020)
#define ZMP_CLK_GATE_SF_RESET_CTRL	(ZMP_PA_SYSCTRL + 0x00FC)
#define ZMP_LCD_PCLK_CTRL			(ZMP_PA_SYSCTRL + 0x0100)
#define ZMP_LCD_DSI_CLK_CFG			(ZMP_PA_SYSCTRL + 0x0074)
#define ZMP_LCD_DSI_LANE_SWAP		(ZMP_PA_SYSCTRL + 0x0078)


/* LCD controller registers */
#define ZMP_LCD_CTRL					(0x20010000)
#define ZMP_LCD_TOP_CONFIG			(ZMP_LCD_CTRL + 0x00)  //LCD Top config
#define ZMP_LCD_MPU_1				(ZMP_LCD_CTRL + 0x04)  //LCD MPU 1
#define ZMP_MPU_READ					(ZMP_LCD_CTRL + 0x0C)  //MPU read
#define ZMP_LCD_RGB_GEN_INFO			(ZMP_LCD_CTRL + 0x10)  //RGB general info cfg
#define ZMP_LCD_RGB_CTRL				(ZMP_LCD_CTRL + 0x14)  //RGB channel ctrl
#define ZMP_RGB_VIRTUAL_LEN			(ZMP_LCD_CTRL + 0x18)  //RGB virtual len
#define ZMP_RGB_VIRTUAL_OFFSET		(ZMP_LCD_CTRL + 0x1C)  //RGB virtual offset
#define ZMP_OSD_ADDRESS				(ZMP_LCD_CTRL + 0x20)  //OSD address
#define ZMP_OSD_OFFSET				(ZMP_LCD_CTRL + 0x24)  //OSD offset
#define ZMP_OSD_PALETTE_ADD			(ZMP_LCD_CTRL + 0x28)  //OSD palette add
#define ZMP_OSD_SIZE_ALPHA			(ZMP_LCD_CTRL + 0x38)  //OSD size alpha
#define ZMP_RGB_BACKGROUND			(ZMP_LCD_CTRL + 0x3C)  //RGB background
#define ZMP_RGB_INTERFACE1			(ZMP_LCD_CTRL + 0x40)  //RGB interface 1
#define ZMP_RGB_INTERFACE2			(ZMP_LCD_CTRL + 0x44)  //RGB interface 2
#define ZMP_RGB_INTERFACE3			(ZMP_LCD_CTRL + 0x48)  //RGB interface 3
#define ZMP_RGB_INTERFACE4			(ZMP_LCD_CTRL + 0x4C)  //RGB interface 4
#define ZMP_RGB_INTERFACE5			(ZMP_LCD_CTRL + 0x50)  //RGB interface 5
#define ZMP_RGB_INTERFACE6			(ZMP_LCD_CTRL + 0x54)  //RGB interface 6
#define ZMP_RGB_INTERFACE7			(ZMP_LCD_CTRL + 0x58)  //RGB interface 7
#define ZMP_RGB_OFFSET				(ZMP_LCD_CTRL + 0x9C)  //RGB offset setting
#define ZMP_RGB_SIZE					(ZMP_LCD_CTRL + 0xA0)  //RGB size
#define ZMP_PANEL_SIZE				(ZMP_LCD_CTRL + 0xA4)  //PANEL size
#define ZMP_REG_CONFIGURE			(ZMP_LCD_CTRL + 0xA8)  //REG configure
#define ZMP_LCD_GO					(ZMP_LCD_CTRL + 0xAC)  //LCD go
#define ZMP_LCD_INT_STATUS			(ZMP_LCD_CTRL + 0xB0)  //LCD status
#define ZMP_LCD_INT_MASK				(ZMP_LCD_CTRL + 0xB4)  //LCD int mask
#define ZMP_LCD_SW_CTRL				(ZMP_LCD_CTRL + 0xB8)  //LCD sw ctrl
#define ZMP_LCD_PCLK					(ZMP_LCD_CTRL + 0xBC)  //LCD pclk

#define ZMP_PACKET_HEAD_REG			(ZMP_LCD_CTRL + 0xD0)  //Packet head register
#define ZMP_TX_PAYLOAD_REG			(ZMP_LCD_CTRL + 0xD4)  //Tx_payload register
#define ZMP_DPI_STATUS_REG			(ZMP_LCD_CTRL + 0xD8)  //status register
#define ZMP_DPI_CONFIG_REG			(ZMP_LCD_CTRL + 0xDC)  //configuration register


/* DSI(Display Serial Interface) config for MIPI LCD */
#define ZMP_DSI_CTRL					(0x20500000)
#define ZMP_DSI_NUM_LANES			(ZMP_DSI_CTRL + 0x00)  //CFG_NUM_LANES register
#define ZMP_DSI_NONCONTINUOUS_CLK	(ZMP_DSI_CTRL + 0x04)  //CFG_NONCONTINUOUS_CLK
#define ZMP_DSI_T_PRE				(ZMP_DSI_CTRL + 0x08)  //CFG_T_PRE register
#define ZMP_DSI_T_POST				(ZMP_DSI_CTRL + 0x0C)  //CFG_T_POST register
#define ZMP_DSI_TX_GAP				(ZMP_DSI_CTRL + 0x10)  //CFG_TX_GAP register
#define ZMP_DSI_AUTOINSERT_EOTP		(ZMP_DSI_CTRL + 0x14)  //CFG_AUTOINSERT_EOTP register
#define ZMP_DSI_EXT_CMDS_AFTER_EOTP	(ZMP_DSI_CTRL + 0x18)  //CFG_EXTRA_CMDS_AFTER_EOTP
#define ZMP_DSI_HTX_TO_COUNT			(ZMP_DSI_CTRL + 0x1C)  //CFG_HTX_TO_COUNT register
#define ZMP_DSI_LRX_H_TO_COUNT		(ZMP_DSI_CTRL + 0x20)  //CFG_LRX_H_TO_COUNT register
#define ZMP_DSI_BTA_H_TO_COUNT		(ZMP_DSI_CTRL + 0x24)  //CFG_BTA_H_TO_COUNT register
#define ZMP_DSI_T_WAKEUP				(ZMP_DSI_CTRL + 0x28)  //CFG_TWAKEUP register
#define ZMP_DSI_STATUS_OUT			(ZMP_DSI_CTRL + 0x2C)  //CFG_STATUS_OUT register
#define ZMP_DSI_PIX_PAYPLOAD_SIZE	(ZMP_DSI_CTRL + 0x200)  //CFG_DPI_PIXEL_PAYLOAD_SIZE register
#define ZMP_DSI_PIX_FIFO_SEND_LEVEL	(ZMP_DSI_CTRL + 0x204)  //CFG_DPI_PIXEL_FIFO_SEND_LEVEL register
#define ZMP_DSI_IF_COLOR_CODING		(ZMP_DSI_CTRL + 0x208)  //CFG_DPI_PIXEL_COLOR_CODING register
#define ZMP_DSI_PIX_FORMAT			(ZMP_DSI_CTRL + 0x20C)  //CFG_DPI_PIXEL_FORMAT register
#define ZMP_DSI_VSYNC_POL			(ZMP_DSI_CTRL + 0x210)  //CFG_DPI_VSYNC_POLARITY register
#define ZMP_DSI_HSYNC_POL			(ZMP_DSI_CTRL + 0x214)  //CFG_DPI_HSYNC_POLARITY register
#define ZMP_DSI_VIDEO_MODE			(ZMP_DSI_CTRL + 0x218)  //CFG_DPI_VIDEO_MODE register
#define ZMP_DSI_HFP					(ZMP_DSI_CTRL + 0x21C)  //CFG_DPI_HFP register
#define ZMP_DSI_HBP					(ZMP_DSI_CTRL + 0x220)  //CFG_DPI_HBP register
#define ZMP_DSI_HSA					(ZMP_DSI_CTRL + 0x224)  //CFG_DPI_HSA register
#define ZMP_DSI_MULT_PKTS_EN			(ZMP_DSI_CTRL + 0x228)  //CFG_DPI_ENABLE_MULT_PKTS register
#define ZMP_DSI_VBP					(ZMP_DSI_CTRL + 0x22C)  //CFG_DPI_VBP register
#define ZMP_DSI_VFP					(ZMP_DSI_CTRL + 0x230)  //CFG_DPI_VFP register
#define ZMP_DSI_BLLP_MODE			(ZMP_DSI_CTRL + 0x234)  //CFG_DPI_BLLP_MODE register
#define ZMP_DSI_USE_NULL_PKT_BLLP	(ZMP_DSI_CTRL + 0x238)  //CFG_DPI_USE_NULL_PKT_BLLP register
#define ZMP_DSI_VACTIVE				(ZMP_DSI_CTRL + 0x23C)  //CFG_DPI_VACTIVE register
#define ZMP_DSI_VC					(ZMP_DSI_CTRL + 0x240)  //CFG_DPI_VC register


#define clear_low_bits(val, len)        ((val) >> (len) << (len))
#define clear_high_bits(val, len)       ((val) << (len) >> (len))
#define mask(msb, lsb)                  clear_high_bits(clear_low_bits(~0UL, (lsb)), (sizeof(~0UL)*8 - 1 - (msb)))
#define lcdc_set_reg(val, reg, msb, lsb) __raw_writel((__raw_readl(reg) & ~mask((msb), (lsb))) | (((val) << (lsb)) & mask((msb), (lsb))), \
						     (reg))
#define True 1
#define False 0

struct lcdc_dsi_info {
	u32 num_lane;
	u32 txd0;
	u32 txd1;
	u32 txd2;
	u32 txd3;
	u32 txd4;

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




//DPHY  DSI TXD 
typedef enum 
{
	LCD_MIPI_DAT0_LANE = 0,
	LCD_MIPI_DAT1_LANE,		
	LCD_MIPI_DAT2_LANE,		
	LCD_MIPI_DAT3_LANE,		
	LCD_MIPI_CLK_LANE,
}T_LCD_MIPI_LANE_DEFINE;

/**
 *  lcd controller output interface definition
 *   IF_MPU: output MPU lcd panel signal
 *   IF_RGB: output RGB lcd panel signal
 *   IF_DSI: output RGB lcd panel signal by DSI interface
 */
enum lcdc_if {
	DISP_IF_RGB = 0,
	DISP_IF_DSI,
	DISP_IF_MPU,
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

struct lcdc_panel_info {
	const char *name;

	dma_addr_t paddr; /* physical address of the buffer */
	void * vaddr; /* virtual address of the buffer */
	u32 paddr_offset;
	u32 size;
	
	u32 rgb_if;
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

	enum lcd_sig_pol pclk_pol;
	enum lcd_sig_pol hsync_pol;
	enum lcd_sig_pol vsync_pol;
	enum lcd_sig_pol vogate_pol;

	/* lcd data bus width */
	enum lcd_rgb_bus_width bus_width;
};


enum lcd_rgb_dma_mode {
	LCD_RGB_DMA_NORMAL_MODE = 0,
	LCD_RGB_DMA_2X_MODE,
};

enum lcd_dummy_seq {
	LCD_DUMMY_BACK = 0,
	LCD_DUMMY_FRONT,
};

enum lcd_rgb_seq_sel {
	LCD_RGB_SEQ_CONSTANT = 0,
	LCD_RGB_SEQ_DIFFERENT,
};

enum lcd_rgb_seq {
	LCD_SEQ_RGB = 0,
	LCD_SEQ_RBG,
	LCD_SEQ_GRB,
	LCD_SEQ_GBR,
	LCD_SEQ_BRG,
	LCD_SEQ_BGR,
};

//RGB LCD 8 bit位宽所需要设置的显示模式
enum lcd_tpg_mode{
	LCD_RGB_TPG052 = 0,
	LCD_RGB_TPG051,
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



struct lcdc_rgb_info {
	u32 alarm_empty;
	u32 alarm_full;

	enum lcd_rgb_dma_mode dma_mode;
	
	enum lcd_dummy_seq dummy_seq; //for 8bit rgb panel settings
	enum lcd_rgb_seq_sel rgb_seq_sel;
	enum lcd_rgb_seq rgb_odd_seq;
	enum lcd_rgb_seq rgb_even_seq;
	enum lcd_tpg_mode tpg_mode;
	u32 blank_sel;
	u32 pos_blank_level;
	u32 neg_blank_level;

	u32 vpage_en;
	u32 vpage_hsize;
	u32 vpage_vsize;
	u32 vpage_hoffset;
	u32 vpage_voffset;
	
	enum lcd_rgb_bg_color bg_color;

	u32 vh_delay_en;
	u32 vh_delay_cycle;
	u32 v_unit;
	
	u32 alert_line;
};

struct lcdc_rgb_input_info {
	/* rgb input data size */
	u32 width;
	u32 height;
	u32 h_offset;
	u32 v_offset;
	u32 fmt0;
	u32 fmt1;
	u32 rgb_seq;
	u32 a_location; //for 32bits input data
	u32 bpp;
};


struct lcdc_osd_info {
	u32 osd_addr;
	u32 size;
	u32 osd_palette_addr;

	u16 palette[16];

	u32 width;
	u32 height;
	u32 alpha;  //  0...8

	u32 startpoint_left;
	u32 startponit_top;
	enum lcd_rgb_dma_mode dma_mode;
};



/**
 * private data struction of lcd driver
 */
struct zmp_fb {
	int irq;
	int reset_pin;

	/* add zmp_fb drive node and lcd pannel node */
	int fb_nobe;   
	int lcd_node;
	const void *fdt_blob;

	struct lcdc_rgb_input_info *rgb_input;
	struct lcdc_rgb_info *rgb_info;
	struct lcdc_dsi_info *dsi_info;
	struct lcdc_panel_info *panel_info;
	struct lcdc_osd_info *osd_info;

	u32 pseudo_palette[16];
	u32 fb_type;  /* fb type: 0 single fb, 1 double fb */
	u32 fb_id;  /* current fb id */
	bool par_change; /* is parameter changed or not */
	bool lcd_go; /* is panel refreshing or not */
};


#endif
