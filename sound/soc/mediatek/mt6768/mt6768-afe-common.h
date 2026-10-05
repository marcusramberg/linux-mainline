/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6768-afe-common.h  --  Mediatek 6768 audio driver definitions
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Michael Hsiao <michael.hsiao@mediatek.com>
 */

#ifndef _MT_6768_AFE_COMMON_H_
#define _MT_6768_AFE_COMMON_H_

#include <linux/clk.h>
#include <sound/soc.h>
#include <linux/list.h>
#include <linux/regmap.h>
#include "mt6768-reg.h"
#include "../common/mtk-base-afe.h"

struct pinctrl;

enum {
	MT6768_MEMIF_DL1,
	MT6768_MEMIF_DL12,
	MT6768_MEMIF_DL2,
	MT6768_MEMIF_DL3,
	MT6768_MEMIF_VUL,
	MT6768_MEMIF_VUL12,
	MT6768_MEMIF_VUL2,
	MT6768_MEMIF_AWB,
	MT6768_MEMIF_AWB2,
	MT6768_MEMIF_NUM,
	MT6768_DAI_ADDA = MT6768_MEMIF_NUM,
	MT6768_DAI_NUM,
};

enum {
	MT6768_IRQ_0,
	MT6768_IRQ_1,
	MT6768_IRQ_2,
	MT6768_IRQ_3,
	MT6768_IRQ_4,
	MT6768_IRQ_5,
	MT6768_IRQ_6,
	MT6768_IRQ_7,
	MT6768_IRQ_11,
	MT6768_IRQ_12,
	MT6768_IRQ_NUM,
};

struct mt6768_afe_private {
	struct clk **clk;
	int stf_positive_gain_db;
	/* adda */
	int mtkaif_protocol;
	int mtkaif_phase_cycle[4];
	int mtkaif_dmic;
	/* pinctrl states for the mtkaif pads */
	struct pinctrl *pinctrl;
};

int mt6768_dai_adda_register(struct mtk_base_afe *afe);

unsigned int mt6768_general_rate_transform(struct device *dev,
					    unsigned int rate);

int mt6768_init_clock(struct mtk_base_afe *afe);
int mt6768_afe_enable_clock(struct mtk_base_afe *afe);
int mt6768_afe_disable_clock(struct mtk_base_afe *afe);

#endif
