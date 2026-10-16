// SPDX-License-Identifier: GPL-2.0
//
// mt6768-afe-clk.c  --  Mediatek 6768 afe clock ctrl
//
// Copyright (c) 2018 MediaTek Inc.
// Author: Michael Hsiao <michael.hsiao@mediatek.com>

#include <linux/clk.h>
#include <linux/device.h>

#include "mt6768-afe-common.h"

enum {
	CLK_INFRA_AUDIO,
	CLK_INFRA_AUDIO_26M,
	CLK_MUX_AUDIO,
	CLK_MUX_AUDIOINTBUS,
	CLK_TOP_SYSPLL1_D4,
	CLK_AFE,
	CLK_CLK26M,
	CLK_NUM
};

static const char *aud_clks[CLK_NUM] = {
	[CLK_INFRA_AUDIO] = "ifr_audio",
	[CLK_INFRA_AUDIO_26M] = "ifr_audio26m",
	[CLK_MUX_AUDIO] = "audio_sel",
	[CLK_MUX_AUDIOINTBUS] = "aud_intbus_sel",
	[CLK_TOP_SYSPLL1_D4] = "syspll1_d4",
	[CLK_AFE] = "aud_afe",
	[CLK_CLK26M] = "clk26m",
};

int mt6768_init_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;
	int i;

	afe_priv->clk = devm_kcalloc(afe->dev, CLK_NUM,
				     sizeof(*afe_priv->clk), GFP_KERNEL);
	if (!afe_priv->clk)
		return -ENOMEM;

	for (i = 0; i < CLK_NUM; i++) {
		afe_priv->clk[i] = devm_clk_get(afe->dev, aud_clks[i]);
		if (IS_ERR(afe_priv->clk[i]))
			return dev_err_probe(afe->dev,
					     PTR_ERR(afe_priv->clk[i]),
					     "failed to get %s\n", aud_clks[i]);
	}

	return 0;
}

int mt6768_afe_enable_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;
	int ret;

	/*
	 * The audio bus clocks run before the infra gate opens: a register
	 * access through an open gate onto a stopped aud_intbus hangs the
	 * SoC, a closed gate just drops it.
	 */
	ret = clk_prepare_enable(afe_priv->clk[CLK_MUX_AUDIO]);
	if (ret)
		return ret;

	/* the ADDA runs on a 26M base clock, keep audio_sel on 26M */
	ret = clk_set_parent(afe_priv->clk[CLK_MUX_AUDIO],
			     afe_priv->clk[CLK_CLK26M]);
	if (ret)
		goto err_disable_mux_audio;

	ret = clk_prepare_enable(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
	if (ret)
		goto err_disable_mux_audio;

	ret = clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
			     afe_priv->clk[CLK_TOP_SYSPLL1_D4]);
	if (ret)
		goto err_disable_intbus;

	ret = clk_prepare_enable(afe_priv->clk[CLK_INFRA_AUDIO_26M]);
	if (ret)
		goto err_disable_intbus;

	ret = clk_prepare_enable(afe_priv->clk[CLK_INFRA_AUDIO]);
	if (ret)
		goto err_disable_infra_26m;

	ret = clk_prepare_enable(afe_priv->clk[CLK_AFE]);
	if (ret)
		goto err_disable_infra;

	return 0;

err_disable_infra:
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AUDIO]);
err_disable_infra_26m:
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AUDIO_26M]);
err_disable_intbus:
	clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
		       afe_priv->clk[CLK_CLK26M]);
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
err_disable_mux_audio:
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIO]);
	return ret;
}

int mt6768_afe_disable_clock(struct mtk_base_afe *afe)
{
	struct mt6768_afe_private *afe_priv = afe->platform_priv;

	/* close the infra gate before the bus clocks stop, see above */
	clk_disable_unprepare(afe_priv->clk[CLK_AFE]);
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AUDIO]);
	clk_disable_unprepare(afe_priv->clk[CLK_INFRA_AUDIO_26M]);
	clk_set_parent(afe_priv->clk[CLK_MUX_AUDIOINTBUS],
		       afe_priv->clk[CLK_CLK26M]);
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIOINTBUS]);
	clk_disable_unprepare(afe_priv->clk[CLK_MUX_AUDIO]);

	return 0;
}
