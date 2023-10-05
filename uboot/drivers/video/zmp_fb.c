/*
 * Copyright (C) 2012 Samsung Electronics
 *
 * Author: InKi Dae <inki.dae@samsung.com>
 * Author: Donghwa Lee <dh09.lee@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <common.h>
#include <lcd.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <asm/io.h>
#include <asm-generic/errno.h>
#include "zmp_fb.h"
#include <ubi_uboot.h>
#include <asm/arch-zmp110x/zmp_cpu.h>
#include <asm-generic/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

//static unsigned int panel_width, panel_height;

extern struct vidinfo panel_info;

struct zmp_fb *fb_dev;

enum PANNEL_IF_TYPE{
	RGB = 0,
	MIPI,
};

/*
 * board_init_f(arch/arm/lib/board.c) calls lcd_setmem() which needs
 * panel_info.vl_col, panel_info.vl_row and panel_info.vl_bpix to reserve
 * FB memory at a very early stage, i.e even before exynos_fimd_parse_dt()
 * is called. So, we are forced to statically assign it.
 * check ok , 10 inch lcd(1280x800), 24bit
 */
vidinfo_t panel_info  = {
	.vl_col = LCD_XRES,
	.vl_row = LCD_YRES,
	.vl_width = LCD_XRES,
	.vl_height = LCD_YRES,
	.vl_bpix = LCD_COLOR24,
};


/* check ok */
unsigned long get_peri_pll_freq(void)
{
	unsigned long pll_m, pll_n, pll_od;
	unsigned long peril_pll_clk;
	unsigned long regval;

	regval = readl(PERIL_PLL_CHANNEL_CTRL_REG);
	pll_od = (regval & (0x3 << 12)) >> 12;
	pll_n = (regval & (0xf << 8)) >> 8;
	pll_m = regval & 0xff;

	peril_pll_clk = (12 * pll_m)/(pll_n * (1 << pll_od));

	if ((pll_od >= 1) && ((pll_n >= 1) && (pll_n <= 12))
		&& ((pll_m >= 84) && (pll_m <= 254)))
		return peril_pll_clk;

	return 0;

}

int zmp_lcd_parameter_parse_dt1(const void *blob, struct zmp_fb *fb_dev)
{
	unsigned int node;
	unsigned int node_sta;
	int depth;
	int len;
	int ret = 0;

	node = fdtdec_next_compatible(blob, 0, COMPAT_ZMP_H3D_LCD);
	//printf("%s, line:%d, blob:0x%x, node:0x%x\n", __func__, __LINE__, blob, node);
	if (node <= 0) {
		printf("zmp_fb: Can't get device node\n");
		ret = 1;
		goto lcd_para_exit;
	}

	fb_dev->fb_nobe = node;


	/* get the lcd panel info from the dts */
	/* parse child dts node, get the panel info and config list */
	depth = 0;
	node_sta = fdtdec_next_compatible_subnode_sta(blob, node, &depth);
	if (node_sta <= 0) {
		printf("zmp_fb: Can't get lcd pannel node for fimd\n");
		ret = 1;
		goto lcd_para_exit;
	}

	fb_dev->lcd_node = node_sta;
	fb_dev->panel_info->name = fdt_getprop(blob, node_sta, "compatible", &len);
	fb_dev->panel_info->rgb_if = fdtdec_get_int(blob, node_sta, "panel-if", 0);
	//printf("%s, line:%d, get panel_info->name:%s, node_sta:0x%x OK!!!\n", __func__, __LINE__, fb_dev->panel_info->name, node_sta);
	//printf("%s, line:%d, get panel_info->rgb_if:%s, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->rgb_if?"MIPI":"RGB");

	/*
	* mipi pannel: has reset pin
	* rgb pannel: no reset pin
	*/
	//if (fb_dev->panel_info->rgb_if == 0x1){
		fb_dev->reset_pin = fdtdec_get_valoffset_int(blob, node, "reset-pins", 1, 0);
		if (!fb_dev->reset_pin) {
			printf("could not get mipi pannel device reset pin\n");
			ret = 1;
			goto lcd_para_exit;
	//	}
		//printf("%s, line:%d, %s pannel if!\n", __func__, __LINE__, fb_dev->panel_info->rgb_if?"MIPI":"RGB");
	//}else{
	//	fb_dev->reset_pin = -1;
		/*fb_dev->reset_pin = fdtdec_get_valoffset_int(blob, node, "reset-pins", 1, 0);
		if (!fb_dev->reset_pin) {
			printf("could not get rgb pannel device reset pin\n");
			ret = 1;
			goto lcd_para_exit;
		}*/
		//printf("%s, line:%d, %s pannel if!\n", __func__, __LINE__, fb_dev->panel_info->rgb_if?"MIPI":"RGB");
	}

	fb_dev->lcd_go = False;
	fb_dev->par_change = False;

	/* get lcd fb type, 0:single buffer, 1:double buffer */
	fb_dev->fb_type = fdtdec_get_int(blob, node, "lcd-fb-type", 0);

	/* get lcdc rgb config info from dts */
	fb_dev->rgb_input->width = fdtdec_get_int(blob, node, "lcd-logo-width", 0);
	fb_dev->rgb_input->height = fdtdec_get_int(blob, node, "lcd-logo-height", 0);
	fb_dev->rgb_input->fmt0 = fdtdec_get_int(blob, node, "lcd-logo-fmt0", 0);
	fb_dev->rgb_input->fmt1 = fdtdec_get_int(blob, node, "lcd-logo-fmt1", 0);
	fb_dev->rgb_input->rgb_seq = fdtdec_get_int(blob, node, "lcd-logo-rgb-seq", 0);
	fb_dev->rgb_input->a_location = fdtdec_get_int(blob, node, "lcd-logo-a-location", 0);
	fb_dev->rgb_info->alarm_empty = fdtdec_get_int(blob, node, "lcd-alarm-empty", 0);
	fb_dev->rgb_info->alarm_full = fdtdec_get_int(blob, node, "lcd-alarm-full", 0);
	fb_dev->rgb_info->dma_mode = fdtdec_get_int(blob, node, "lcd-dma-mode", 0);
	fb_dev->rgb_info->dummy_seq = fdtdec_get_int(blob, node, "lcd-dummy-seq", 0);
	fb_dev->rgb_info->rgb_seq_sel = fdtdec_get_int(blob, node, "lcd-rgb-seq-sel", 0);
	fb_dev->rgb_info->rgb_odd_seq = fdtdec_get_int(blob, node, "lcd-rgb-odd-seq", 0);
	fb_dev->rgb_info->rgb_even_seq = fdtdec_get_int(blob, node, "lcd-rgb-even-seq", 0);
	fb_dev->rgb_info->tpg_mode = fdtdec_get_int(blob, node, "lcd-rgb-tpg-sel", 0);
	fb_dev->rgb_info->vpage_en = fdtdec_get_int(blob, node, "lcd-vpage-en", 0);
	fb_dev->rgb_info->blank_sel = fdtdec_get_int(blob, node, "lcd-blank-sel", 0);
	fb_dev->rgb_info->pos_blank_level = fdtdec_get_int(blob, node, "lcd-pos-blank-level", 0);
	fb_dev->rgb_info->neg_blank_level = fdtdec_get_int(blob, node, "lcd-neg-blank-level", 0);
	fb_dev->rgb_info->bg_color = fdtdec_get_int(blob, node, "lcd-rgb-bg-color", 0);
	fb_dev->rgb_info->vh_delay_en = fdtdec_get_int(blob, node, "lcd-vh-delay-en", 0);
	fb_dev->rgb_info->vh_delay_cycle = fdtdec_get_int(blob, node, "lcd-vh-delay-cycle", 0);
	fb_dev->rgb_info->v_unit = fdtdec_get_int(blob, node, "lcd-v-unit", 0);
	fb_dev->rgb_info->alert_line = fdtdec_get_int(blob, node, "lcd-alert-line", 0);

#if 0
	printf("%s,line:%d, fdt_addr:0x%x\n", __func__, __LINE__, blob);
	printf("%s, line:%d, get device node OK, node:%d!!!\n", __func__, __LINE__, node);
	printf("%s, line:%d, get device reset pin:%d, OK!!!\n", __func__, __LINE__, fb_dev->reset_pin);
	printf("%s, line:%d, get device fb_type:%d, OK!!!\n", __func__, __LINE__, fb_dev->fb_type);
	printf("%s, line:%d, get device rgb_input->width:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->width);
	printf("%s, line:%d, get device rgb_input->height:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->height);
	printf("%s, line:%d, get device rgb_input->fmt0:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->fmt0);
	printf("%s, line:%d, get device rgb_input->fmt1:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->fmt1);
	printf("%s, line:%d, get device rgb_input->rgb_seq:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->rgb_seq);
	printf("%s, line:%d, get device rgb_input->a_location:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_input->a_location);
	printf("%s, line:%d, get device rgb_info->alarm_empty:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->alarm_empty);
	printf("%s, line:%d, get device rgb_info->alarm_full:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->alarm_full);
	printf("%s, line:%d, get device rgb_info->dma_mode:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->dma_mode);
	printf("%s, line:%d, get device rgb_info->dummy_seq:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->dummy_seq);
	printf("%s, line:%d, get device rgb_info->rgb_seq_sel:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->rgb_seq_sel);
	printf("%s, line:%d, get device rgb_info->rgb_odd_seq:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->rgb_odd_seq);
	printf("%s, line:%d, get device rgb_info->rgb_even_seq:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->rgb_even_seq);
	printf("%s, line:%d, get device rgb_info->tpg_mode:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->tpg_mode);
	printf("%s, line:%d, get device rgb_info->vpage_en:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->vpage_en);
	printf("%s, line:%d, get device rgb_info->blank_sel:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->blank_sel);
	printf("%s, line:%d, get device rgb_info->pos_blank_level:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->pos_blank_level);
	printf("%s, line:%d, get device rgb_info->neg_blank_level:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->neg_blank_level);
	printf("%s, line:%d, get device rgb_info->bg_color:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->bg_color);
	printf("%s, line:%d, get device rgb_info->vh_delay_en:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->vh_delay_en);
	printf("%s, line:%d, get device rgb_info->vh_delay_cycle:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->vh_delay_cycle);
	printf("%s, line:%d, get device rgb_info->v_unit:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->v_unit);
	printf("%s, line:%d, get device rgb_info->alert_line:%d, OK!!!\n", __func__, __LINE__, fb_dev->rgb_info->alert_line);
	printf("%s, line:%d, get panel_info->name:%s, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->name);
	printf("%s, line:%d, get panel_info->rgb_if:%s, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->rgb_if?"MIPI":"RGB");
#endif
	ret = 0;
lcd_para_exit:

	return ret;

}


int zmp_lcd_parameter_parse_dt2(struct zmp_fb *fb_dev)
{
	unsigned int node_sta;
	const void *blob;

	blob = fb_dev->fdt_blob;
	node_sta = fb_dev->lcd_node;

	fb_dev->panel_info->rgb_seq = fdtdec_get_int(blob, node_sta, "panel-rgb-seq", 0);
	fb_dev->panel_info->width = fdtdec_get_int(blob, node_sta, "panel-width", 0);
	fb_dev->panel_info->height = fdtdec_get_int(blob, node_sta, "panel-height", 0);
	fb_dev->panel_info->pclk_div = fdtdec_get_int(blob, node_sta, "panel-pclk-div", 0);
	fb_dev->dsi_info->num_lane = fdtdec_get_int(blob, node_sta, "panel-dsi-num-lane", 0);
	fb_dev->dsi_info->txd0 = fdtdec_get_int(blob, node_sta, "panel-dsi-txd0", 0);
	fb_dev->dsi_info->txd1 = fdtdec_get_int(blob, node_sta, "panel-dsi-txd1", 0);
	fb_dev->dsi_info->txd2 = fdtdec_get_int(blob, node_sta, "panel-dsi-txd2", 0);
	fb_dev->dsi_info->txd3 = fdtdec_get_int(blob, node_sta, "panel-dsi-txd3", 0);
	fb_dev->dsi_info->txd4 = fdtdec_get_int(blob, node_sta, "panel-dsi-txd4", 0);
	fb_dev->dsi_info->noncontinuous_clk = fdtdec_get_int(blob, node_sta, "panel-dsi-noncontinuous-clk", 0);
	fb_dev->dsi_info->t_pre = fdtdec_get_int(blob, node_sta, "panel-dsi-t-pre", 0);
	fb_dev->dsi_info->t_post = fdtdec_get_int(blob, node_sta, "panel-dsi-t-post", 0);
	fb_dev->dsi_info->tx_gap = fdtdec_get_int(blob, node_sta, "panel-dsi-tx-gap", 0);
	fb_dev->dsi_info->autoinsert_eotp = fdtdec_get_int(blob, node_sta, "panel-dsi-autoinsert-eotp", 0);
	fb_dev->dsi_info->htx_to_count = fdtdec_get_int(blob, node_sta, "panel-dsi-htx-to-count", 0);
	fb_dev->dsi_info->lrx_to_count = fdtdec_get_int(blob, node_sta, "panel-dsi-lrx-to-count", 0);
	fb_dev->dsi_info->bta_to_count = fdtdec_get_int(blob, node_sta, "panel-dsi-bta-to-count", 0);
	fb_dev->dsi_info->t_wakeup = fdtdec_get_int(blob, node_sta, "panel-dsi-t-wakeup", 0);
	fb_dev->dsi_info->pix_fifo_level = fdtdec_get_int(blob, node_sta, "panel-dsi-pix-fifo-send-level", 0);
	fb_dev->dsi_info->if_color_coding = fdtdec_get_int(blob, node_sta, "panel-dsi-if-color-coding", 0);
	fb_dev->dsi_info->pix_format = fdtdec_get_int(blob, node_sta, "panel-dsi-pix-format", 0);
	fb_dev->dsi_info->vsync_pol = fdtdec_get_int(blob, node_sta, "panel-dsi-vsync-pol", 0);
	fb_dev->dsi_info->hsync_pol = fdtdec_get_int(blob, node_sta, "panel-dsi-hsync-pol", 0);
	fb_dev->dsi_info->video_mode = fdtdec_get_int(blob, node_sta, "panel-dsi-video-mode", 0);
	fb_dev->dsi_info->hfp = fdtdec_get_int(blob, node_sta, "panel-dsi-hfp", 0);
	fb_dev->dsi_info->hbp = fdtdec_get_int(blob, node_sta, "panel-dsi-hbp", 0);
	fb_dev->dsi_info->hsa = fdtdec_get_int(blob, node_sta, "panel-dsi-hsa", 0);
	fb_dev->dsi_info->mult_pkts_en = fdtdec_get_int(blob, node_sta, "panel-dsi-mult-pkts-en", 0);
	fb_dev->dsi_info->vbp = fdtdec_get_int(blob, node_sta, "panel-dsi-vbp", 0);
	fb_dev->dsi_info->vfp = fdtdec_get_int(blob, node_sta, "panel-dsi-vfp", 0);
	fb_dev->dsi_info->vsa = fdtdec_get_int(blob, node_sta, "panel-dsi-vsa", 0);
	fb_dev->dsi_info->bllp_mode = fdtdec_get_int(blob, node_sta, "panel-dsi-bllp-mode", 0);
	fb_dev->dsi_info->use_null_pkt_bllp = fdtdec_get_int(blob, node_sta, "panel-dsi-use-null-pkt-bllp", 0);
	fb_dev->dsi_info->vc = fdtdec_get_int(blob, node_sta, "panel-dsi-vc", 0);
	fb_dev->dsi_info->cmd_type = fdtdec_get_int(blob, node_sta, "panel-dsi-cmd-type", 0);
	fb_dev->dsi_info->dsi_clk = fdtdec_get_int(blob, node_sta, "panel-dsi-clk", 0);

#if 0
	printf("%s,line:%d, fdt_addr:0x%x\n", __func__, __LINE__, blob);
	printf("%s, line:%d, get panel_info->rgb_seq:%d, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->rgb_seq);
	printf("%s, line:%d, get panel_info->width:%d, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->width);
	printf("%s, line:%d, get panel_info->height:%d, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->height);
	printf("%s, line:%d, get panel_info->pclk_div:%d, OK!!!\n", __func__, __LINE__, fb_dev->panel_info->pclk_div);
	printf("%s, line:%d, get dsi_info->num_lane:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->num_lane);
	printf("%s, line:%d, get dsi_info->txd0:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->txd0);
	printf("%s, line:%d, get dsi_info->txd1:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->txd1);
	printf("%s, line:%d, get dsi_info->txd2:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->txd2);
	printf("%s, line:%d, get dsi_info->txd3:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->txd3);
	printf("%s, line:%d, get dsi_info->txd4:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->txd4);
	printf("%s, line:%d, get dsi_info->noncontinuous_clk:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->noncontinuous_clk);
	printf("%s, line:%d, get dsi_info->t_pre:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->t_pre);
	printf("%s, line:%d, get dsi_info->t_post:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->t_post);
	printf("%s, line:%d, get dsi_info->tx_gap:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->tx_gap);
	printf("%s, line:%d, get dsi_info->autoinsert_eotp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->autoinsert_eotp);
	printf("%s, line:%d, get dsi_info->htx_to_count:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->htx_to_count);
	printf("%s, line:%d, get dsi_info->lrx_to_count:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->lrx_to_count);
	printf("%s, line:%d, get dsi_info->bta_to_count:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->bta_to_count);
	printf("%s, line:%d, get dsi_info->t_wakeup:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->t_wakeup);
	printf("%s, line:%d, get dsi_info->pix_fifo_level:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->pix_fifo_level);
	printf("%s, line:%d, get dsi_info->if_color_coding:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->if_color_coding);
	printf("%s, line:%d, get dsi_info->pix_format:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->pix_format);
	printf("%s, line:%d, get dsi_info->vsync_pol:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->vsync_pol);
	printf("%s, line:%d, get dsi_info->hsync_pol:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->hsync_pol);
	printf("%s, line:%d, get dsi_info->video_mode:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->video_mode);
	printf("%s, line:%d, get dsi_info->hfp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->hfp);
	printf("%s, line:%d, get dsi_info->hbp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->hbp);
	printf("%s, line:%d, get dsi_info->hsa:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->hsa);
	printf("%s, line:%d, get dsi_info->mult_pkts_en:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->mult_pkts_en);
	printf("%s, line:%d, get dsi_info->vbp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->vbp);
	printf("%s, line:%d, get dsi_info->vfp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->vfp);
	printf("%s, line:%d, get dsi_info->vsa:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->vsa);
	printf("%s, line:%d, get dsi_info->bllp_mode:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->bllp_mode);
	printf("%s, line:%d, get dsi_info->use_null_pkt_bllp:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->use_null_pkt_bllp);
	printf("%s, line:%d, get dsi_info->vc:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->vc);
	printf("%s, line:%d, get dsi_info->cmd_type:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->cmd_type);
	printf("%s, line:%d, get dsi_info->dsi_clk:%d, OK!!!\n", __func__, __LINE__, fb_dev->dsi_info->dsi_clk);
#endif

	return 0;
}

int zmp_lcd_parameter_parse_dt3(struct zmp_fb *fb_dev)
{
	struct lcdc_panel_info *panel_info = fb_dev->panel_info;
	unsigned int node_sta;
	const void *blob;

	blob = fb_dev->fdt_blob;
	node_sta = fb_dev->lcd_node;

	panel_info->rgb_seq = fdtdec_get_int(blob, node_sta, "panel-rgb-seq", 0);
	panel_info->width = fdtdec_get_int(blob, node_sta, "panel-width", 0);
	panel_info->height = fdtdec_get_int(blob, node_sta, "panel-height", 0);
	panel_info->pclk_div = fdtdec_get_int(blob, node_sta, "panel-pclk-div", 0);
	panel_info->thpw = fdtdec_get_int(blob, node_sta, "panel-thpw", 0);
	panel_info->thb = fdtdec_get_int(blob, node_sta, "panel-thb", 0);
	panel_info->thf = fdtdec_get_int(blob, node_sta, "panel-thf", 0);
	panel_info->tvpw = fdtdec_get_int(blob, node_sta, "panel-tvpw", 0);
	panel_info->tvb = fdtdec_get_int(blob, node_sta, "panel-tvb", 0);
	panel_info->tvf = fdtdec_get_int(blob, node_sta, "panel-tvf", 0);
	panel_info->pclk_pol = fdtdec_get_int(blob, node_sta, "panel-pclk-pol", 0);
	panel_info->hsync_pol = fdtdec_get_int(blob, node_sta, "panel-hsync-pol", 0);
	panel_info->vsync_pol = fdtdec_get_int(blob, node_sta, "panel-vsync-pol", 0);
	panel_info->vogate_pol = fdtdec_get_int(blob, node_sta, "panel-vogate-pol", 0);
	panel_info->bus_width = fdtdec_get_int(blob, node_sta, "panel-bus-width", 0);

#if 0
	printf("%s,line:%d, fdt_addr:0x%x\n", __func__, __LINE__, blob);
	printf("%s, line:%d, get panel_info->rgb_seq:%d, OK!!!\n", __func__, __LINE__, panel_info->rgb_seq);
	printf("%s, line:%d, get panel_info->width:%d, OK!!!\n", __func__, __LINE__, panel_info->width);
	printf("%s, line:%d, get panel_info->height:%d, OK!!!\n", __func__, __LINE__, panel_info->height);
	printf("%s, line:%d, get panel_info->pclk_div:%d, OK!!!\n", __func__, __LINE__, panel_info->pclk_div);
	printf("%s, line:%d, get panel_info->thpw:%d, OK!!!\n", __func__, __LINE__, panel_info->thpw);
	printf("%s, line:%d, get panel_info->thb:%d, OK!!!\n", __func__, __LINE__, panel_info->thb);
	printf("%s, line:%d, get panel_info->thf:%d, OK!!!\n", __func__, __LINE__, panel_info->thf);
	printf("%s, line:%d, get panel_info->tvpw:%d, OK!!!\n", __func__, __LINE__, panel_info->tvpw);
	printf("%s, line:%d, get panel_info->tvb:%d, OK!!!\n", __func__, __LINE__, panel_info->tvb);
	printf("%s, line:%d, get panel_info->tvf:%d, OK!!!\n", __func__, __LINE__, panel_info->tvf);
	printf("%s, line:%d, get panel_info->pclk_pol:%d, OK!!!\n", __func__, __LINE__, panel_info->pclk_pol);
	printf("%s, line:%d, get panel_info->hsync_pol:%d, OK!!!\n", __func__, __LINE__, panel_info->hsync_pol);
	printf("%s, line:%d, get panel_info->vsync_pol:%d, OK!!!\n", __func__, __LINE__, panel_info->vsync_pol);
	printf("%s, line:%d, get panel_info->vogate_pol:%d, OK!!!\n", __func__, __LINE__, panel_info->vogate_pol);
	printf("%s, line:%d, get panel_info->bus_width:%d, OK!!!\n", __func__, __LINE__, panel_info->bus_width);
#endif

	return 0;
}


/* check ok */
static void mipi_clk_and_pad_setting(struct zmp_fb *fb_dev)
{
	u32 regval;
	struct lcdc_dsi_info *dsi_info = fb_dev->dsi_info;

	/* lane pad swap setting 0x6a100000, check ok */
	regval = __raw_readl(ZMP_LCD_DSI_LANE_SWAP);
	regval &= ~((0x7<<17)|(0x7<<20)|(0x7<<23)|(0x7<<26)|(0x7<<29));
	regval |= (((dsi_info->txd0)
		|(dsi_info->txd1<<3)
		|(dsi_info->txd2<<6)
		|(dsi_info->txd3<<9)
		|(dsi_info->txd4<<12))<<17);
	__raw_writel(regval, ZMP_LCD_DSI_LANE_SWAP);

	/* enable mipi dsi clk 0xf000b, check ok */
	regval = __raw_readl(ZMP_CLK_GATE_SF_RESET_CTRL);
	regval &= ~(0x1<<2);
	__raw_writel(regval, ZMP_CLK_GATE_SF_RESET_CTRL);

	/* reset mipi dsi controller clk, check ok */
	regval = __raw_readl(ZMP_CLK_GATE_SF_RESET_CTRL);
	regval |= (0x1<<18);
	__raw_writel(regval, ZMP_CLK_GATE_SF_RESET_CTRL);
	mdelay(1);
	regval = __raw_readl(ZMP_CLK_GATE_SF_RESET_CTRL);
	regval &= ~(0x1<<18);
	__raw_writel(regval, ZMP_CLK_GATE_SF_RESET_CTRL);
	mdelay(10);

	/* reset pclk and hold pclk 0x4000c700, k 0x4008c702 check ok */
	regval = __raw_readl(ZMP_LCD_PCLK_CTRL);
	regval |= (0x1<<30);
	__raw_writel(regval, ZMP_LCD_PCLK_CTRL);

	/* DSI Controller & DPHY clock gate enable 0x40000000, k 0x400000b8 check ok   */
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval &= ~(0x1<<31);
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);

	/* DSI DPHY Reset, check ok  */
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval &= ~(0x1<<30);
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);
	mdelay(1);
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval |= (0x1<<30);
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);

	/* DSI Controller TxByteClkHS, DSI Controller & dphy TxClkEsc reset, check ok  */
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval |= (0x3<<28);
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);
	mdelay(1);
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval &= ~(0x3<<28);
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);

	/* check if the DPHY is ready, check ok   */
	do {
	} while (!(__raw_readl(ZMP_DPI_STATUS_REG) & (0x1<<2)));

#if 0
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_DSI_LANE_SWAP, __raw_readl(ZMP_LCD_DSI_LANE_SWAP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_CLK_GATE_SF_RESET_CTRL, __raw_readl(ZMP_CLK_GATE_SF_RESET_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_CLK_GATE_SF_RESET_CTRL, __raw_readl(ZMP_CLK_GATE_SF_RESET_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_PCLK_CTRL, __raw_readl(ZMP_LCD_PCLK_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_DSI_CLK_CFG, __raw_readl(ZMP_LCD_DSI_CLK_CFG));
#endif

}

/* check ok */
static void mipi_dsi_cfg(struct zmp_fb *fb_dev)
{
	struct lcdc_dsi_info *dsi_info = fb_dev->dsi_info;

	/* mipi dsi config reg setting 3, check ok */
	__raw_writel(dsi_info->num_lane, ZMP_DSI_NUM_LANES);

	/* mipi dsi config reg setting 0 , check ok */
	__raw_writel(dsi_info->noncontinuous_clk, ZMP_DSI_NONCONTINUOUS_CLK);

	/* mipi dsi config reg setting 1 , check ok */
	__raw_writel(dsi_info->t_pre, ZMP_DSI_T_PRE);

	/* mipi dsi config reg setting 1 , check ok */
	__raw_writel(dsi_info->t_post, ZMP_DSI_T_POST);

	/* mipi dsi config reg setting 1 , check ok */
	__raw_writel(dsi_info->tx_gap, ZMP_DSI_TX_GAP);

	/* mipi dsi config reg setting 0 , check ok */
	__raw_writel(dsi_info->autoinsert_eotp, ZMP_DSI_AUTOINSERT_EOTP);

	/* mipi dsi config reg setting 0xFFFFFF , check ok */
	__raw_writel(dsi_info->htx_to_count, ZMP_DSI_HTX_TO_COUNT);

	/* mipi dsi config reg setting 0xFFFFFF , check ok */
	__raw_writel(dsi_info->lrx_to_count, ZMP_DSI_LRX_H_TO_COUNT);

	/* mipi dsi config reg setting 0xFFFFFF , check ok */
	__raw_writel(dsi_info->bta_to_count, ZMP_DSI_BTA_H_TO_COUNT);

	/* mipi dsi config reg setting 0xc8 , check ok */
	__raw_writel(dsi_info->t_wakeup, ZMP_DSI_T_WAKEUP);

	/* mipi dsi config reg setting 0x320 , check ok */
	__raw_writel(dsi_info->pix_payload_size, ZMP_DSI_PIX_PAYPLOAD_SIZE);

	/* mipi dsi config reg setting 0x200 , check ok */
	__raw_writel(dsi_info->pix_fifo_level, ZMP_DSI_PIX_FIFO_SEND_LEVEL);

	/* mipi dsi config reg setting 0x5 , check ok */
	__raw_writel(dsi_info->if_color_coding, ZMP_DSI_IF_COLOR_CODING);

	/* mipi dsi config reg setting 0x3 , check ok */
	__raw_writel(dsi_info->pix_format, ZMP_DSI_PIX_FORMAT);

	/* mipi dsi config reg setting 0x0 , check ok */
	__raw_writel(dsi_info->vsync_pol, ZMP_DSI_VSYNC_POL);

	/* mipi dsi config reg setting 0x0 , check ok */
	__raw_writel(dsi_info->hsync_pol, ZMP_DSI_HSYNC_POL);

	/* mipi dsi config reg setting 0x2 , check ok */
	__raw_writel(dsi_info->video_mode, ZMP_DSI_VIDEO_MODE);

	/* re-cacult hfp, hbp, hsa */
	/* mipi dsi config reg setting 0xB5 , check ok */
	__raw_writel(dsi_info->hfp, ZMP_DSI_HFP);

	/* mipi dsi config reg setting 0x5A , check ok */
	__raw_writel(dsi_info->hbp, ZMP_DSI_HBP);

	/* mipi dsi config reg setting 0x5A , check ok */
	__raw_writel(dsi_info->hsa, ZMP_DSI_HSA);

	/* mipi dsi config reg setting 0x0 , check ok */
	__raw_writel(dsi_info->mult_pkts_en, ZMP_DSI_MULT_PKTS_EN);

	/* re-cacult vbp, vfp */
	/* mipi dsi config reg setting 0x8 , check ok */
	__raw_writel(dsi_info->vbp, ZMP_DSI_VBP);

	/* mipi dsi config reg setting 0x8 , check ok */
	__raw_writel(dsi_info->vfp, ZMP_DSI_VFP);

	/* mipi dsi config reg setting 0x1 , check ok */
	__raw_writel(dsi_info->bllp_mode, ZMP_DSI_BLLP_MODE);

	/* mipi dsi config reg setting 0x0 , check ok */
	__raw_writel(dsi_info->use_null_pkt_bllp, ZMP_DSI_USE_NULL_PKT_BLLP);

	/* mipi dsi config reg setting 0x500 , check ok */
	__raw_writel(dsi_info->vactive, ZMP_DSI_VACTIVE);

	/* mipi dsi config reg setting 0x0 , check ok */
	__raw_writel(dsi_info->vc, ZMP_DSI_VC);

#if 0
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_NUM_LANES, __raw_readl(ZMP_DSI_NUM_LANES));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_NONCONTINUOUS_CLK, __raw_readl(ZMP_DSI_NONCONTINUOUS_CLK));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_T_PRE, __raw_readl(ZMP_DSI_T_PRE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_T_POST, __raw_readl(ZMP_DSI_T_POST));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_TX_GAP, __raw_readl(ZMP_DSI_TX_GAP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_AUTOINSERT_EOTP, __raw_readl(ZMP_DSI_AUTOINSERT_EOTP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_HTX_TO_COUNT, __raw_readl(ZMP_DSI_HTX_TO_COUNT));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_LRX_H_TO_COUNT, __raw_readl(ZMP_DSI_LRX_H_TO_COUNT));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_BTA_H_TO_COUNT, __raw_readl(ZMP_DSI_BTA_H_TO_COUNT));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_T_WAKEUP, __raw_readl(ZMP_DSI_T_WAKEUP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_PIX_PAYPLOAD_SIZE, __raw_readl(ZMP_DSI_PIX_PAYPLOAD_SIZE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_PIX_FIFO_SEND_LEVEL, __raw_readl(ZMP_DSI_PIX_FIFO_SEND_LEVEL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_IF_COLOR_CODING, __raw_readl(ZMP_DSI_IF_COLOR_CODING));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_PIX_FORMAT, __raw_readl(ZMP_DSI_PIX_FORMAT));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VSYNC_POL, __raw_readl(ZMP_DSI_VSYNC_POL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_HSYNC_POL, __raw_readl(ZMP_DSI_HSYNC_POL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VIDEO_MODE, __raw_readl(ZMP_DSI_VIDEO_MODE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_HFP, __raw_readl(ZMP_DSI_HFP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_HBP, __raw_readl(ZMP_DSI_HBP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_HSA, __raw_readl(ZMP_DSI_HSA));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_MULT_PKTS_EN, __raw_readl(ZMP_DSI_MULT_PKTS_EN));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VBP, __raw_readl(ZMP_DSI_VBP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VFP, __raw_readl(ZMP_DSI_VFP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_BLLP_MODE, __raw_readl(ZMP_DSI_BLLP_MODE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_USE_NULL_PKT_BLLP, __raw_readl(ZMP_DSI_USE_NULL_PKT_BLLP));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VACTIVE, __raw_readl(ZMP_DSI_VACTIVE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_DSI_VC, __raw_readl(ZMP_DSI_VC));
#endif


}


/* check ok */
static void mipi_panel_cmd_setting(struct zmp_fb *fb_dev)
{
	int num_inits;
	u32 config, delay = 0;
	u32 payload_type = 0;
	u32 short_data;
	u8 /*temp_i,temp,*/data_type,remainder,quotient;
	u8 long_packet[256];
	u8 short_packet[2];
	u8 *p;
	/*u32 count;*/
	u32 regval;
	int i,j/*,k*/;
	int len = 0;
	unsigned int node;
	const void *blob;

	blob = fb_dev->fdt_blob;
	node = fb_dev->lcd_node;
	fdt_getprop(blob, node, "panel-init-list", &len);
	num_inits = len ? (len / 4) : 0;
	debug("%s, len:%d, num_inits:%d\n", __func__, len, num_inits);

	/* check if the DPHY is ready , check ok */
	do {
	} while (!(__raw_readl(ZMP_DPI_STATUS_REG) & (1<<2)));

	for(i = 0; i < num_inits; ) {
		for(j = 0;; j++) {
			config = fdtdec_get_valoffset_int(blob, node, "panel-init-list", i+j, 0);
			//printf("index:i=%d, j=%d, (i+j)=%d, config:%x\n", i,j, i+j, config);

			if (j == 0)
				/* delay parameter */
				delay = (u8)config;
			else if (j == 1) {
				/* data type */
				data_type = (u8) config;
				//printf("data_type = %x\n", data_type);
				/* 0x03 Generic Short WRITE, no parameters Short
				   0x13  Generic Short WRITE, 1 parameter Short
				   0x23 Generic Short WRITE, 2 parameters Short
				   0x05 DCS Short WRITE, no parameters Short
				   0x15 DCS Short WRITE, 1 parameter Short
				   0x29 Generic Long Write Long
				   0x39 DCS Long Write/write_LUT Command Packet Long
				   */
				if (data_type == 0x03 || data_type == 0x13 || data_type == 0x23
						|| data_type == 0x05 || data_type == 0x15) {
					/* configure short packet cmd */
					payload_type = 1;
					regval = fb_dev->dsi_info->cmd_type|(0x1<<1); // short packet cmd
				} else if (data_type == 0x29 || data_type == 0x39) {
					/* configure long packet cmd  */
					payload_type = 0;
					regval = fb_dev->dsi_info->cmd_type|(0x0<<1); // long packet cmd
				}
				__raw_writel(regval, ZMP_DPI_CONFIG_REG);
			}else {
				/*zzp, 2020/5/27 translating to char(1 bytes) here. */
				if (config != 0xFFF) {
					if (payload_type == 1)
						short_packet[j - 2] = (unsigned char)config;

					if (payload_type == 0)
						long_packet[j - 2] = (unsigned char)config;
				}

				/*zzp, 2020/5/27 start to send packet. */
				if(config == 0xFFF){
					/* sending short packet */
					if (payload_type == 1) {
						regval = (data_type|(fb_dev->dsi_info->vc<<6)|(short_packet[0]<<8)|(short_packet[1]<<16));
						__raw_writel(regval, ZMP_PACKET_HEAD_REG);

						/* check if the current packet tx done */
						do {
						} while (!(__raw_readl(ZMP_DPI_STATUS_REG) & 0x1));
						mdelay(delay);
					}

					/* sending long packet  */
					if (payload_type == 0) {
						p = long_packet;
						/* write long packet data to payload register. 4bytes every time. */
						for(quotient=0;quotient<(j-2)/4;quotient++){
							short_data = *(unsigned long *)p;
							p += 4;
							__raw_writel(short_data, ZMP_TX_PAYLOAD_REG);
						}

						remainder = (j-2)%4;
						if(remainder !=0){
							short_data = 0;
							memcpy(&short_data,p,remainder);
							__raw_writel(short_data, ZMP_TX_PAYLOAD_REG);
						}

						/*pr_err("data_type %x count %d vc %d \n", data_type, j-2, fb_dev->dsi_info->vc);*/
						regval = (data_type|(fb_dev->dsi_info->vc<<6)|((j-2)<<8));
						__raw_writel(regval, ZMP_PACKET_HEAD_REG);
						/* check if the current packet tx done */
						do {
						} while (!(__raw_readl(ZMP_DPI_STATUS_REG) & 0x1));
						mdelay(delay);
					}
				}
			}
			if(config == 0xFFF)
				break;
		}
		i += (j + 1);
	}
	debug("panel-init-list end...\n");
}

/*
according lcd dtb panel-bus-width parameter to reset sharepin
LCD_PWM
LCD_RESET

RGB_PCLK       		(GPIO74_PU) 	REG4 BIT[13:12]=01
RGB_VOHSYNC 		(GPIO73_PU)		REG4 BIT[11:10]=01
RGB_VOVSYNC 		(GPIO72_PU) 	REG4 BIT[9:8]=01
RGB_VOGATE 			(GPIO71_PU)  	REG4 BIT[7:6]=01

LCD_D0				(GPIO75_PU) 	REG4 BIT[15:14]=01
LCD_D1  			(GPIO76_PU) 	REG4 BIT[17:16]=01
LCD_D2  			(GPIO77_PU)		REG4 BIT[19:18]=01
LCD_D3  			(GPIO78_PU)		REG4 BIT[21:20]=01
LCD_D4  			(GPIO79_PU)		REG4 BIT[23:22]=01
LCD_D5  			(GPIO80_PU)		REG4 BIT[25:24]=01
LCD_D6  			(GPIO81_PU)		REG4 BIT[27:26]=01
LCD_D7  			(GPIO82_PU)		REG4 BIT[29:28]=01

LCD_D8				(GPIO83_PU)		REG4 BIT[31:30]=01
LCD_D9 				(GPIO84_PU)		REG5 BIT[1:0]=01
LCD_D10 			(GPIO85_PU)		REG5 BIT[3:2]=01
LCD_D11				(GPIO86_PU)		REG5 BIT[5:4]=01
LCD_D12				(GPIO87_PU)		REG5 BIT[7:6]=01
LCD_D13				(GPIO88_PU)		REG5 BIT[9:8]=01
LCD_D14				(GPIO89_PU)		REG5 BIT[11:10]=01
LCD_D15				(GPIO90_PU)		REG5 BIT[13:12]=01

8bit mode: LCD0~LCD7
16bit mode(RGB565):LCD[15:11]=R[4:0], LCD[10:5]=G[5:0], LCD[4:0]=B[4:0]
18bit mode(RGB666):LCD[17:12]=R[5:0], LCD[11:6]=G[5:0], LCD[5:0]=B[5:0]
24bit mode(RGB888):LCD[23:16]=R[7:0], LCD[15:8]=G[7:0], LCD[7:0]=B[7:0]
*/
static void rgb_pannel_sharepin_cfg(struct zmp_fb *fb_dev)
{
	u32 regval;
	unsigned int node_sta;
	const void *blob;

	blob = fb_dev->fdt_blob;
	node_sta = fb_dev->lcd_node;

	fb_dev->panel_info->bus_width = fdtdec_get_int(blob, node_sta, "panel-bus-width", 0);
	if(fb_dev->panel_info->bus_width == LCD_RGB_BUS_WIDTH_8BIT){
		/*
		* 8bit mode: LCD0~LCD7, use GPIO75~GPIO82
		* unknown use which gpio as i2c scl & sda,here no set i2c sharepin
		*/
		regval = __raw_readl(SHARE_PIN_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28));
		__raw_writel(regval, SHARE_PIN_CFG4_REG);


		/*
		* cfg rgb share driver strength to maxest 01:10.5mA for B version chip
		*/
		regval = __raw_readl(PAD_DRV_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x1<<17)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x0<<17)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28));
		__raw_writel(regval, PAD_DRV_CFG4_REG);

	}else if(fb_dev->panel_info->bus_width == LCD_RGB_BUS_WIDTH_16BIT){
		/*
		* 16bit mode(RGB565):LCD[15:11]=R[4:0], LCD[10:5]=G[5:0], LCD[4:0]=B[4:0]
		* unknown use which gpio as i2c scl & sda,here no set i2c sharepin
		*/
		regval = __raw_readl(SHARE_PIN_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x1<<30));
		__raw_writel(regval, SHARE_PIN_CFG4_REG);

		regval = __raw_readl(SHARE_PIN_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12));
		regval |= ((0x1<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12));
		__raw_writel(regval, SHARE_PIN_CFG5_REG);

		/*
		* cfg rgb share driver strength to maxest 01:10.5mA for B version chip
		*/
		regval = __raw_readl(PAD_DRV_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x1<<17)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x0<<17)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x1<<30));
		__raw_writel(regval, PAD_DRV_CFG4_REG);

		regval = __raw_readl(PAD_DRV_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12));
		regval |= ((0x1<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12));
		__raw_writel(regval, PAD_DRV_CFG5_REG);
	}else if(fb_dev->panel_info->bus_width == LCD_RGB_BUS_WIDTH_18BIT){
		/*
		* 18bit mode(RGB666):LCD[17:12]=R[5:0], LCD[11:6]=G[5:0], LCD[5:0]=B[5:0]
		* unknown use which gpio as i2c scl & sda,here no set i2c sharepin
		*/
		regval = __raw_readl(SHARE_PIN_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x2<<30));
		__raw_writel(regval, SHARE_PIN_CFG4_REG);

		regval = __raw_readl(SHARE_PIN_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
			   |(0x3<<14)|(0x3<<16));
		regval |= ((0x2<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16));
		__raw_writel(regval, SHARE_PIN_CFG5_REG);

		/*
		* cfg rgb share driver strength to maxest 01:10.5mA for B version chip
		*/
		regval = __raw_readl(PAD_DRV_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x1<<17)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x0<<17)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x0<<30));
		__raw_writel(regval, PAD_DRV_CFG4_REG);

		regval = __raw_readl(PAD_DRV_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
			   |(0x3<<14)|(0x3<<16));
		regval |= ((0x0<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16));
		__raw_writel(regval, PAD_DRV_CFG5_REG);

	}else if(fb_dev->panel_info->bus_width == LCD_RGB_BUS_WIDTH_24BIT){
		/*
		* 24bit mode(RGB888):LCD[23:16]=R[7:0], LCD[15:8]=G[7:0], LCD[7:0]=B[7:0]
		* usr LCD_D8(gpio83) & LCD_D9(gpio84) as i2c scl & sda for touch screen
		*/
		regval = __raw_readl(SHARE_PIN_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x2<<30));
		__raw_writel(regval, SHARE_PIN_CFG4_REG);

		regval = __raw_readl(SHARE_PIN_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
			   |(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)|(0x3<<28));
		regval |= ((0x2<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)|(0x1<<28));
		__raw_writel(regval, SHARE_PIN_CFG5_REG);

		/*
		* cfg rgb share driver strength to maxest 01:10.5mA for B version chip
		*/
		regval = __raw_readl(PAD_DRV_CFG4_REG);
		regval &= ~((0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
				|(0x3<<14)|(0x1<<17)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)
				|(0x3<<28) |(0x3<<30));
		regval |= ((0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x0<<17)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)
			   |(0x1<<28) |(0x0<<30));
		__raw_writel(regval, PAD_DRV_CFG4_REG);

		regval = __raw_readl(PAD_DRV_CFG5_REG);
		regval &= ~((0x3<<0)
			   |(0x3<<2)|(0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10)|(0x3<<12)
			   |(0x3<<14)|(0x3<<16)|(0x3<<18)|(0x3<<20)|(0x3<<22)|(0x3<<24)|(0x3<<26)|(0x3<<28));
		regval |= ((0x0<<0)
			   |(0x1<<2)|(0x1<<4)|(0x1<<6)|(0x1<<8)|(0x1<<10)|(0x1<<12)
			   |(0x1<<14)|(0x1<<16)|(0x1<<18)|(0x1<<20)|(0x1<<22)|(0x1<<24)|(0x1<<26)|(0x1<<28));
		__raw_writel(regval, PAD_DRV_CFG5_REG);

	}else{
		printf("Err:no lcd sharepin parameter!\n");
		return ;
	}

	/*
	* LCD backlight control GPIO27_PU enable output high lighted backlight
	*/

	mdelay(1);

}


/* check ok */
static void rgb_panel_info_cfg(struct zmp_fb *fb_dev)
{
	struct lcdc_panel_info *panel_info = fb_dev->panel_info;
	struct lcdc_rgb_info *rgb_info = fb_dev->rgb_info;
	u32 regval;
	u32 thlen, tvlen;

	/* top register config and disable rgb and osd channel */
	regval = __raw_readl(ZMP_LCD_TOP_CONFIG);
	regval &= ~((0x1)|(0xF<<3)|(0x1<<11)|(0x3FF<<14)|(0xFF<<24));
	regval|= (((!panel_info->pclk_pol)<<4)|(0x2<<5)
		|((!panel_info->rgb_seq)<<11)
		|(rgb_info->alarm_full<<14)
		|(rgb_info->alarm_empty<<24));
	__raw_writel(regval, ZMP_LCD_TOP_CONFIG);

	/* lcd rgb general info parameter config */
	regval = __raw_readl(ZMP_LCD_RGB_GEN_INFO);
	regval = ((panel_info->vogate_pol)
		|(panel_info->vsync_pol<<1)
		|(panel_info->hsync_pol<<2)
		|(rgb_info->neg_blank_level<<3)
		|(rgb_info->pos_blank_level<<11)
		|(rgb_info->blank_sel<<19)
		|(rgb_info->dma_mode<<20)
		|(panel_info->bus_width<<21)
		|(rgb_info->tpg_mode<<23)
		|(rgb_info->rgb_even_seq<<24)
		|(rgb_info->rgb_odd_seq<<27)
		|(rgb_info->rgb_seq_sel<<30)
		|(rgb_info->dummy_seq<<31));
	__raw_writel(regval, ZMP_LCD_RGB_GEN_INFO);

	/*disable vpage function*/
	regval = __raw_readl(ZMP_LCD_RGB_CTRL);
	regval &= ~(0x1<<30);
	__raw_writel(regval, ZMP_LCD_RGB_CTRL);


	/* set rgb bg color */
	regval = __raw_readl(ZMP_RGB_BACKGROUND);
	regval &= ~(0xFFFFFF);
	regval |= (fb_dev->rgb_info->bg_color);
	__raw_writel(regval, ZMP_RGB_BACKGROUND);


	/* rgb timing setting */
	regval = __raw_readl(ZMP_RGB_INTERFACE1);
	regval &= ~(0xFFFFFF);
	regval |= ((panel_info->tvpw)
		|(panel_info->thpw<<12));
	__raw_writel(regval, ZMP_RGB_INTERFACE1);

	regval = __raw_readl(ZMP_RGB_INTERFACE2);
	regval &= ~(0xFFFFFF);
	regval |= ((panel_info->thd)
		|(panel_info->thb<<12));
	__raw_writel(regval, ZMP_RGB_INTERFACE2);

	thlen = panel_info->thpw
		+ panel_info->thb
		+ panel_info->thd
		+ panel_info->thf;
	regval = __raw_readl(ZMP_RGB_INTERFACE3);
	regval &= ~(0xFFFFFF);
	regval |= ((thlen)
		|(panel_info->thf<<13));
	__raw_writel(regval, ZMP_RGB_INTERFACE3);

	regval = __raw_readl(ZMP_RGB_INTERFACE4);
	regval &= ~(0xFFF);
	regval |= (panel_info->tvb);
	__raw_writel(regval, ZMP_RGB_INTERFACE4);

	regval = __raw_readl(ZMP_RGB_INTERFACE5);
	regval &= ~(0xFFF);
	regval |= (panel_info->tvf);
	__raw_writel(regval, ZMP_RGB_INTERFACE5);

	regval = __raw_readl(ZMP_RGB_INTERFACE6);
	regval &= ~((0x1FFF)|(0x1FFF<<14));
	regval |= ((rgb_info->v_unit)
		|(rgb_info->vh_delay_cycle<<1)
		|(rgb_info->vh_delay_en<<14)
		|(panel_info->tvd<<15));
	__raw_writel(regval, ZMP_RGB_INTERFACE6);

	if(rgb_info->v_unit == 0) {
		tvlen = panel_info->tvpw
			+ panel_info->tvb
			+ panel_info->tvd
			+ panel_info->tvf;
	} else {
		tvlen = panel_info->tvb
			+ panel_info->tvd
			+ panel_info->tvf;
	}
	regval = __raw_readl(ZMP_RGB_INTERFACE7);
	regval &= ~(0x1FFF);
	regval |= (tvlen);
	__raw_writel(regval, ZMP_RGB_INTERFACE7);

	/* set panel size */
	regval = __raw_readl(ZMP_PANEL_SIZE);
	regval &= ~(0x3FFFFF);
	regval |= ((panel_info->height)
		|(panel_info->width<<11));
	__raw_writel(regval, ZMP_PANEL_SIZE);

	/* lcd sw ctrl settings */
	rgb_info->alert_line = panel_info->height - rgb_info->alert_line;
	regval = __raw_readl(ZMP_LCD_SW_CTRL);
	regval &= ~((0xFFF));
	regval |= ((rgb_info->alert_line)|(0x0<<11));
	__raw_writel(regval, ZMP_LCD_SW_CTRL);

	/* pclk domain reset, div setting and enable */
	lcdc_set_reg(1, ZMP_LCD_PCLK_CTRL, 30, 30);
	lcdc_set_reg(0, ZMP_LCD_PCLK_CTRL, 30, 30);

	regval = __raw_readl(ZMP_LCD_PCLK);
	regval &= ~(0x1FF);
	regval |= (0x1)|(panel_info->pclk_div<<1)|(0x1<<8);
	__raw_writel(regval, ZMP_LCD_PCLK);

	/* enable open backlight, output high */
//	gpio_direction_output(CONFIG_LCD_BACKLIGHT_GPIO, 1);
//	gpio_set_pin_level(CONFIG_LCD_BACKLIGHT_GPIO, 1);
	mdelay(1);
#if 0
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_TOP_CONFIG, __raw_readl(ZMP_LCD_TOP_CONFIG));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_RGB_GEN_INFO, __raw_readl(ZMP_LCD_RGB_GEN_INFO));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_RGB_CTRL, __raw_readl(ZMP_LCD_RGB_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_BACKGROUND, __raw_readl(ZMP_RGB_BACKGROUND));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE1, __raw_readl(ZMP_RGB_INTERFACE1));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE2, __raw_readl(ZMP_RGB_INTERFACE2));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE3, __raw_readl(ZMP_RGB_INTERFACE3));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE4, __raw_readl(ZMP_RGB_INTERFACE4));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE5, __raw_readl(ZMP_RGB_INTERFACE5));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE6, __raw_readl(ZMP_RGB_INTERFACE6));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_INTERFACE7, __raw_readl(ZMP_RGB_INTERFACE7));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_PANEL_SIZE, __raw_readl(ZMP_PANEL_SIZE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_SW_CTRL, __raw_readl(ZMP_LCD_SW_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_PCLK_CTRL, __raw_readl(ZMP_LCD_PCLK_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_PCLK, __raw_readl(ZMP_LCD_PCLK));
#endif

}


static void mipi_panel_init(struct zmp_fb *fb_dev)
{
	struct lcdc_panel_info *panel_info = fb_dev->panel_info;
	struct lcdc_dsi_info *dsi_info = fb_dev->dsi_info;
	u32 peri_rate;
	u32 pclk,factor;
	u32 regval;

	debug("%s, line:%d\n", __func__, __LINE__);
	zmp_lcd_parameter_parse_dt2(fb_dev);

	if (dsi_info->mult_pkts_en == 0){
		debug("%s, line:%d\n", __func__, __LINE__);
		dsi_info->pix_payload_size = panel_info->width;
	}
	else if (dsi_info->mult_pkts_en == 1){
		debug("%s, line:%d\n", __func__, __LINE__);
		dsi_info->pix_payload_size = panel_info->width/2;
	}
	dsi_info->vactive = panel_info->height;
	debug("%s, line:%d\n", __func__, __LINE__);
	mipi_clk_and_pad_setting(fb_dev);
	debug("%s, line:%d\n", __func__, __LINE__);

	if (dsi_info->if_color_coding <= 2)
		panel_info->bus_width = 1;
	else if (dsi_info->if_color_coding <= 4)
		panel_info->bus_width = 2;
	else if (dsi_info->if_color_coding <= 5)
		panel_info->bus_width = 3;
	else
		panel_info->bus_width = 3;
	debug("%s, line:%d, bus_width:%d\n", __func__, __LINE__, panel_info->bus_width);
	peri_rate = get_peri_pll_freq();
	if(peri_rate != 600) {
		peri_rate = 600;
		printf("err, peril pll is not 600MHz!\n");
	}

	pclk = peri_rate/(panel_info->pclk_div + 1);

	//dpi_pixel_size * dpi_event_size * dpi_pclk_period = number_of_dsi_bytes * 8 * hs_bit_period / cfg_num_lanes
	//dpi_event_size = number_of_dsi_bytes*8*dpi_clk/(cfg_num_lanes*hs_clk)
	//
	//factor = (1000*8*600)/((panel_info->pclk_div + 1)*(dsi_info->num_lane+1)*dsi_info->dsi_clk);
	factor = (1000*8*pclk)/((dsi_info->num_lane+1)*dsi_info->dsi_clk);

	/* get RGB timing config from dts */
	panel_info->pclk_pol = 0;
	panel_info->vsync_pol = dsi_info->vsync_pol?0:1;
	panel_info->hsync_pol = dsi_info->hsync_pol?0:1;
	panel_info->thpw = dsi_info->hsa;
	panel_info->thb = dsi_info->hbp;
	panel_info->thd = panel_info->width;
	panel_info->thf = dsi_info->hfp;
	panel_info->tvpw = dsi_info->vsa;
	panel_info->tvb = dsi_info->vbp;
	panel_info->tvd = panel_info->height;
	panel_info->tvf = dsi_info->vfp;

	/* calcuate mipi dsi timing setting */
	dsi_info->hsa = panel_info->thpw*1000/factor;
	dsi_info->hbp = panel_info->thb*1000/factor;
	dsi_info->hfp = panel_info->thf*1000/factor;
	dsi_info->vbp = panel_info->tvb;
	dsi_info->vfp = panel_info->tvf;

	mipi_dsi_cfg(fb_dev);

	/* set mipi high speed clk u 0x40000092, k 0x400000b8 */
	regval = __raw_readl(ZMP_LCD_DSI_CLK_CFG);
	regval &= ~(0x1FF);
	if (dsi_info->dsi_clk <= 160)
		regval |= ((dsi_info->dsi_clk-80)*8/10); // 1.25*
	else if (dsi_info->dsi_clk <= 320)
		regval |= ((dsi_info->dsi_clk)*2/5); // 2.5*
	else
		regval |= ((dsi_info->dsi_clk)/5 + 64); // 5*
	__raw_writel(regval, ZMP_LCD_DSI_CLK_CFG);
	debug("mipi clk reg value :%x\n", regval);

	/* reset mipi lcd panel */
	if (fb_dev->reset_pin > 0) {
		debug("%s, line:%d\n", __func__, __LINE__);
		gpio_direction_output(fb_dev->reset_pin, 1);
		gpio_set_pin_level(fb_dev->reset_pin, 1);
		mdelay(1);
		gpio_set_pin_level(fb_dev->reset_pin, 0);
		mdelay(10);
		gpio_set_pin_level(fb_dev->reset_pin, 1);
		mdelay(10);
	}

	debug("%s, line:%d\n", __func__, __LINE__);
	mipi_panel_cmd_setting(fb_dev);

	debug("%s, line:%d\n", __func__, __LINE__);
	rgb_panel_info_cfg(fb_dev);

	debug("%s, line:%d\n", __func__, __LINE__);
}



static void rgb_panel_init(struct zmp_fb *fb_dev)
{
	struct lcdc_panel_info *panel_info = fb_dev->panel_info;

	zmp_lcd_parameter_parse_dt3(fb_dev);
	panel_info->thd = panel_info->width;
	panel_info->tvd = panel_info->height;


	/* reset rgb lcd panel */
	if (fb_dev->reset_pin > 0) {
		debug("%s, line:%d\n", __func__, __LINE__);
		gpio_direction_output(fb_dev->reset_pin, 1);
		gpio_set_pin_level(fb_dev->reset_pin, 1);
		mdelay(1);
		gpio_set_pin_level(fb_dev->reset_pin, 0);
		mdelay(10);
		gpio_set_pin_level(fb_dev->reset_pin, 1);
		mdelay(10);
	}

	rgb_panel_info_cfg(fb_dev);
}

static void lcdc_info_cfg(struct zmp_fb *fb_dev)
{
	if(fb_dev->panel_info->rgb_if == DISP_IF_RGB){
		/*
		* cfg rgb pannel share pin
		*/
		rgb_pannel_sharepin_cfg(fb_dev);

		/*
		* rgb ctroller cfg
		*/
		rgb_panel_init(fb_dev);
	}else if(fb_dev->panel_info->rgb_if == DISP_IF_DSI){
		struct lcdc_dsi_info *dsi_info;
		dsi_info = kzalloc(sizeof(struct lcdc_dsi_info), GFP_KERNEL);
		if (!dsi_info){
			printf("alloc lcdc dsi info buffer error\n");
			return ;
		}
		else {
		 	fb_dev->dsi_info = dsi_info;
			mipi_panel_init(fb_dev);
		}
	}else if(fb_dev->panel_info->rgb_if == DISP_IF_MPU){
		/* add special init for mpu panel */
		/* todo */
	}

}


/* set rgb display for logo display
*
*  rgb channel config and rgb go
*/
static void lcdc_rgb_display(struct zmp_fb *fb_dev)
{
	u32 regval;

	/* input data format is determined by bit[8] and bit[13] of this register.
	 * 00:input data format is 16bits (RGB565 or BGR565)
	 * 01:input data format is 24bits (RGB888 or BGR888)
     * 10 or 11:input data format is 32bits (ARGB888 ABGR888 RGBA888 BGRA888)
	 * bit[12] input data seq, 0:RGB,1:BGR
	 * bit[7] inout a location, 0: A front, 1, A back
	 */
	regval = __raw_readl(ZMP_LCD_TOP_CONFIG);
	regval &= ~((0x3<<7)|(0x3<<12));
	regval |= ((fb_dev->rgb_input->a_location<<7)
		|(fb_dev->rgb_input->fmt1<<8)
		|(fb_dev->rgb_input->rgb_seq<<12)
		|(fb_dev->rgb_input->fmt0<<13));
	__raw_writel(regval, ZMP_LCD_TOP_CONFIG);

	/* config RGB input size */
	regval = __raw_readl(ZMP_RGB_SIZE);
	regval &= ~(0x3FFFFF);
	regval|= ((fb_dev->rgb_input->height)
		|(fb_dev->rgb_input->width<<11));
	__raw_writel(regval, ZMP_RGB_SIZE);

	/* config RGB input offset */
	regval = __raw_readl(ZMP_RGB_OFFSET);
	regval &= ~(0x3FFFFF);
	regval|= ((fb_dev->rgb_input->v_offset)
		|(fb_dev->rgb_input->h_offset<<11));
	__raw_writel(regval, ZMP_RGB_OFFSET);

    /* config RGB dma address */
	///fb_dev->panel_info->paddr = 0x03d02000;
	fb_dev->panel_info->paddr_offset = 0x0;
	regval = __raw_readl(ZMP_LCD_RGB_CTRL);
	regval &= ~(0x3FFFFFFF);
	regval|= ((fb_dev->panel_info->paddr + fb_dev->panel_info->paddr_offset)&0x3FFFFFFF);
	__raw_writel(regval, ZMP_LCD_RGB_CTRL);

	/* enable rgb channel */
	regval = __raw_readl(ZMP_LCD_TOP_CONFIG);
	regval |= (0x1<<3);
	__raw_writel(regval, ZMP_LCD_TOP_CONFIG);

	/* set sw ctrl en */
	regval = __raw_readl(ZMP_LCD_SW_CTRL);
	regval|= (0x1<<12);
	__raw_writel(regval, ZMP_LCD_SW_CTRL);

	/* rgb go */
	if (fb_dev->lcd_go == False)
	{
		if (fb_dev->panel_info->rgb_if == DISP_IF_MPU) {
			__raw_writel(0x1<<3, ZMP_LCD_GO);
			fb_dev->lcd_go = True;
		} else {
			__raw_writel(0x1<<2, ZMP_LCD_GO);
			fb_dev->lcd_go = True;
		}
	}

#if 0
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_TOP_CONFIG, __raw_readl(ZMP_LCD_TOP_CONFIG));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_SIZE, __raw_readl(ZMP_RGB_SIZE));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_RGB_OFFSET, __raw_readl(ZMP_RGB_OFFSET));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_RGB_CTRL, __raw_readl(ZMP_LCD_RGB_CTRL));
	printf("panel_info->width=%d\n",fb_dev->panel_info->width);
	printf("panel_info->height=%d\n",fb_dev->panel_info->height);
	printf("rgb_info->width=%d\n",fb_dev->rgb_input->width);
	printf("rgb_info->height=%d\n",fb_dev->rgb_input->height);
	printf("rgb_info->v_offset=%d\n",fb_dev->rgb_input->v_offset);
	printf("rgb_info->h_offset=%d\n",fb_dev->rgb_input->h_offset);
	printf("panel_info->paddr=0x%x, \n",fb_dev->panel_info->paddr);
	printf("panel_info->paddr_offset=0x%x, \n",fb_dev->panel_info->paddr_offset);
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_TOP_CONFIG, __raw_readl(ZMP_LCD_TOP_CONFIG));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_SW_CTRL, __raw_readl(ZMP_LCD_SW_CTRL));
	printf("%s, reg:0x%x, val:0x%x\n", __func__, ZMP_LCD_GO, __raw_readl(ZMP_LCD_GO));
#endif


}

extern void zmp_pwm_init(unsigned long long duty_ns, unsigned long long period_ns);

static void lcd_panel_on(struct zmp_fb *fb_dev)
{
    int i;
    unsigned int value;
	/* lcd logo display */
	lcdc_rgb_display(fb_dev);

	mdelay(150);
//backlight
	#if CONFIG_LCD_BACKLIGHT_PWM
	zmp_pwm_init(10000000,10000000);
	#else
	gpio_direction_output(CONFIG_LCD_BACKLIGHT_GPIO, 1);
	gpio_set_pin_level(CONFIG_LCD_BACKLIGHT_GPIO, 1);
	#endif
}


void lcd_ctrl_init(void *lcdbase)
{
	u32 regval;
	u32 ret = 0;
	struct lcdc_rgb_input_info *rgb_input;
	struct lcdc_rgb_info *rgb_info;
	struct lcdc_osd_info *osd_info;
	struct lcdc_panel_info *panel_info;

	/*
	* malloc fb some struct space
	*/
	rgb_input = kzalloc(sizeof(struct lcdc_rgb_input_info), GFP_KERNEL);
	rgb_info = kzalloc(sizeof(struct lcdc_rgb_info), GFP_KERNEL);
	osd_info = kzalloc(sizeof(struct lcdc_osd_info), GFP_KERNEL);
	panel_info = kzalloc(sizeof(struct lcdc_panel_info), GFP_KERNEL);
	if ((!rgb_input)||(!rgb_info)||(!osd_info)||(!panel_info)) {
		printf("alloc lcdc info buffer error\n");
		ret = -ENOMEM;
		return ;
	}

	/*
	* malloc fb_dev struct space
	*/
	fb_dev = malloc(sizeof(struct zmp_fb));
	if(!fb_dev){
		printf("alloc lcdc info buffer2 error\n");
		ret = -ENOMEM;
		return ;
	}

	fb_dev->rgb_input = rgb_input;
	fb_dev->rgb_info = rgb_info;
	fb_dev->osd_info = osd_info;
	fb_dev->panel_info = panel_info;
	fb_dev->panel_info->paddr = (dma_addr_t)(lcdbase - 0x80000000);
	//debug("panel_info->paddr=0x%x, \n",fb_dev->panel_info->paddr);

	/*
	* enable lcd controller clk
	*/
	regval = __raw_readl(ZMP_CLK_GATE_CTRL0);
	regval &= ~(0x1<<22);
	__raw_writel(regval, ZMP_CLK_GATE_CTRL0);

	/*
	* reset lcd controller clk
	*/
	regval = __raw_readl(ZMP_SW_RESET_CTRL0);
	regval |= (0x1<<22);
	__raw_writel(regval, ZMP_SW_RESET_CTRL0);
	mdelay(1);
	regval = __raw_readl(ZMP_SW_RESET_CTRL0);
	regval &= ~(0x1<<22);
	__raw_writel(regval, ZMP_SW_RESET_CTRL0);
	mdelay(10);

	/*
	* init fb dev fdt base address
	*/
	fb_dev->fdt_blob = gd->fdt_blob;
	ret = zmp_lcd_parameter_parse_dt1(gd->fdt_blob, fb_dev);
	if(ret){
		printf("err, get dtb and parse lcd parameter!\n");
		ret = -ENOMEM;
		return ;
	}

	lcdc_info_cfg(fb_dev);
}

void lcd_enable(void)
{
	/* copy logo file here */
	//memcpy(panel_info->vaddr, boot_logo, sizeof(boot_logo));
	fb_dev->fb_id = 0;
	fb_dev->panel_info->paddr_offset = 0;

	/* set the display offset */
	fb_dev->rgb_input->h_offset = (fb_dev->panel_info->width-fb_dev->rgb_input->width)/2;
	fb_dev->rgb_input->h_offset -= fb_dev->rgb_input->h_offset%2;
	fb_dev->rgb_input->v_offset = (fb_dev->panel_info->height-fb_dev->rgb_input->height)/2;
	fb_dev->rgb_input->v_offset -= fb_dev->rgb_input->v_offset%2;

	/* set rgb display for logo display */
	lcd_panel_on(fb_dev);
}

/* dummy function */
void lcd_setcolreg(ushort regno, ushort red, ushort green, ushort blue)
{
	return;
}
