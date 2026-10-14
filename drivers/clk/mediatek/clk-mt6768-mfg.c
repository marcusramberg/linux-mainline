// SPDX-License-Identifier: GPL-2.0-only

#include <dt-bindings/clock/mediatek,mt6768-clk.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mfgcfg_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define MFG_ASYNC_CON		0x20
#define MFG_ASYNC_CON_1		0x24
#define MFG_GLOBAL_CON		0xb0

/* one mfgcfg instance */
static struct regmap *mfgcfg_regmap;
static struct clk_ops mfgcfg_bg3d_ops;

/*
 * The vendor driver writes these after every BG3D enable; they reset with
 * MFG_ASYNC, which genpd powers off in system suspend.
 */
static int mfgcfg_bg3d_enable(struct clk_hw *hw)
{
	int ret = mtk_clk_gate_ops_setclr.enable(hw);

	if (ret)
		return ret;

	regmap_clear_bits(mfgcfg_regmap, MFG_GLOBAL_CON, BIT(8) | BIT(10));
	regmap_set_bits(mfgcfg_regmap, MFG_ASYNC_CON, GENMASK(25, 22));
	regmap_set_bits(mfgcfg_regmap, MFG_ASYNC_CON_1, BIT(0));

	return 0;
}

static const struct mtk_gate mfgcfg_clks[] = {
	GATE_MTK(CLK_MFGCFG_BG3D, "mfgcfg_bg3d", "mfg_ck", &mfgcfg_cg_regs, 0,
		 &mfgcfg_bg3d_ops),
};

static const struct mtk_clk_desc mfg_desc = {
	.clks = mfgcfg_clks,
	.num_clks = ARRAY_SIZE(mfgcfg_clks),
};

static const struct of_device_id of_match_clk_mt6768_mfg[] = {
	{ .compatible = "mediatek,mt6768-mfgcfg", .data = &mfg_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6768_mfg);

static int clk_mt6768_mfg_probe(struct platform_device *pdev)
{
	mfgcfg_regmap = device_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(mfgcfg_regmap))
		return PTR_ERR(mfgcfg_regmap);

	mfgcfg_bg3d_ops = mtk_clk_gate_ops_setclr;
	mfgcfg_bg3d_ops.enable = mfgcfg_bg3d_enable;

	return mtk_clk_simple_probe(pdev);
}

static struct platform_driver clk_mt6768_mfg = {
	.driver = {
		.name = "clk-mt6768-mfg",
		.of_match_table = of_match_clk_mt6768_mfg,
	},
	.probe = clk_mt6768_mfg_probe,
	.remove = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt6768_mfg);

MODULE_DESCRIPTION("MediaTek MT6768 mfg clocks driver");
MODULE_LICENSE("GPL");

