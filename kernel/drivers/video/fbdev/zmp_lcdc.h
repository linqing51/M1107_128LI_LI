/*
 * ZMP1107 lcd driver
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

#ifndef _ZMP1107_LCD_REGS_H_
#define _ZMP1107_LCD_REGS_H_

// CIS SCLK  & LCD PCLK CFG REGISTER
#define ZMP_PERI_PLL   	(ZMP_VA_SYSCTRL + 0x0014)
#define ZMP_SHAREPIN   	(ZMP_VA_SYSCTRL + 0x0188)

#define ZMP_LCD_CLOCK_GATE   	    (ZMP_VA_SYSCTRL + 0x001C)
#define ZMP_LCD_SW_RESET_CTRL	    (ZMP_VA_SYSCTRL + 0x0020)
#define ZMP_LCD_PCLK_CTRL		    (ZMP_VA_SYSCTRL + 0x0100)
#define ZMP_LCD_DSI_CLK_CFG		    (ZMP_VA_SYSCTRL + 0x0074)
#define ZMP_LCD_DSI_LANE_SWAP	    (ZMP_VA_SYSCTRL + 0x0078)
#define ZMP_CLK_GATE_AND_SOFT_RST	(ZMP_VA_SYSCTRL + 0x00FC)

/* LCD controller registers */
#define ZMP_LCD_TOP_CONFIG			(0x00)  //LCD Top config
#define ZMP_LCD_MPU_1				(0x04)  //LCD MPU 1
#define ZMP_MPU_READ				(0x0C)  //MPU read
#define ZMP_LCD_RGB_GEN_INFO		(0x10)  //RGB general info cfg
#define ZMP_LCD_RGB_CTRL			(0x14)  //RGB channel ctrl
#define ZMP_RGB_VIRTUAL_LEN			(0x18)  //RGB virtual len
#define ZMP_RGB_VIRTUAL_OFFSET		(0x1C)  //RGB virtual offset
#define ZMP_OSD_ADDRESS				(0x20)  //OSD address
#define ZMP_OSD_OFFSET				(0x24)  //OSD offset
#define ZMP_OSD_PALETTE_ADD			(0x28)  //OSD palette add
#define ZMP_OSD_SIZE_ALPHA			(0x38)  //OSD size alpha
#define ZMP_RGB_BACKGROUND			(0x3C)  //RGB background
#define ZMP_RGB_INTERFACE1			(0x40)  //RGB interface 1
#define ZMP_RGB_INTERFACE2			(0x44)  //RGB interface 2
#define ZMP_RGB_INTERFACE3			(0x48)  //RGB interface 3
#define ZMP_RGB_INTERFACE4			(0x4C)  //RGB interface 4
#define ZMP_RGB_INTERFACE5			(0x50)  //RGB interface 5
#define ZMP_RGB_INTERFACE6			(0x54)  //RGB interface 6
#define ZMP_RGB_INTERFACE7			(0x58)  //RGB interface 7
#define ZMP_RGB_OFFSET				(0x9C)  //RGB offset setting
#define ZMP_RGB_SIZE					(0xA0)  //RGB size
#define ZMP_PANEL_SIZE				(0xA4)  //PANEL size
#define ZMP_REG_CONFIGURE			(0xA8)  //REG configure
#define ZMP_LCD_GO					(0xAC)  //LCD go
#define ZMP_LCD_INT_STATUS			(0xB0)  //LCD status
#define ZMP_LCD_INT_MASK				(0xB4)  //LCD int mask
#define ZMP_LCD_SW_CTRL				(0xB8)  //LCD sw ctrl
#define ZMP_LCD_PCLK					(0xBC)  //LCD pclk

#define ZMP_PACKET_HEAD_REG			(0xD0)  //Packet head register
#define ZMP_TX_PAYLOAD_REG			(0xD4)  //Tx_payload register
#define ZMP_DPI_STATUS_REG			(0xD8)  //status register
#define ZMP_DPI_CONFIG_REG			(0xDC)  //configuration register

/* DSI(Display Serial Interface) config for MIPI LCD */
#define ZMP_DSI_NUM_LANES			(0x00)  //CFG_NUM_LANES register
#define ZMP_DSI_NONCONTINUOUS_CLK	(0x04)  //CFG_NONCONTINUOUS_CLK
#define ZMP_DSI_T_PRE				(0x08)  //CFG_T_PRE register
#define ZMP_DSI_T_POST				(0x0C)  //CFG_T_POST register
#define ZMP_DSI_TX_GAP				(0x10)  //CFG_TX_GAP register
#define ZMP_DSI_AUTOINSERT_EOTP		(0x14)  //CFG_AUTOINSERT_EOTP register
#define ZMP_DSI_EXT_CMDS_AFTER_EOTP	(0x18)  //CFG_EXTRA_CMDS_AFTER_EOTP
#define ZMP_DSI_HTX_TO_COUNT			(0x1C)  //CFG_HTX_TO_COUNT register
#define ZMP_DSI_LRX_H_TO_COUNT		(0x20)  //CFG_LRX_H_TO_COUNT register
#define ZMP_DSI_BTA_H_TO_COUNT		(0x24)  //CFG_BTA_H_TO_COUNT register
#define ZMP_DSI_T_WAKEUP				(0x28)  //CFG_TWAKEUP register
#define ZMP_DSI_STATUS_OUT			(0x2C)  //CFG_STATUS_OUT register
#define ZMP_DSI_PIX_PAYPLOAD_SIZE	(0x200)  //CFG_DPI_PIXEL_PAYLOAD_SIZE register
#define ZMP_DSI_PIX_FIFO_SEND_LEVEL	(0x204)  //CFG_DPI_PIXEL_FIFO_SEND_LEVEL register
#define ZMP_DSI_IF_COLOR_CODING		(0x208)  //CFG_DPI_PIXEL_COLOR_CODING register
#define ZMP_DSI_PIX_FORMAT			(0x20C)  //CFG_DPI_PIXEL_FORMAT register
#define ZMP_DSI_VSYNC_POL			(0x210)  //CFG_DPI_VSYNC_POLARITY register
#define ZMP_DSI_HSYNC_POL			(0x214)  //CFG_DPI_HSYNC_POLARITY register
#define ZMP_DSI_VIDEO_MODE			(0x218)  //CFG_DPI_VIDEO_MODE register
#define ZMP_DSI_HFP					(0x21C)  //CFG_DPI_HFP register
#define ZMP_DSI_HBP					(0x220)  //CFG_DPI_HBP register
#define ZMP_DSI_HSA					(0x224)  //CFG_DPI_HSA register
#define ZMP_DSI_MULT_PKTS_EN			(0x228)  //CFG_DPI_ENABLE_MULT_PKTS register
#define ZMP_DSI_VBP					(0x22C)  //CFG_DPI_VBP register
#define ZMP_DSI_VFP					(0x230)  //CFG_DPI_VFP register
#define ZMP_DSI_BLLP_MODE			(0x234)  //CFG_DPI_BLLP_MODE register
#define ZMP_DSI_USE_NULL_PKT_BLLP	(0x238)  //CFG_DPI_USE_NULL_PKT_BLLP register
#define ZMP_DSI_VACTIVE				(0x23C)  //CFG_DPI_VACTIVE register
#define ZMP_DSI_VC					(0x240)  //CFG_DPI_VC register

#endif	/* _ZMP1107_LCD_REGS_H_ */
