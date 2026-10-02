// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 * Copyright (c) 2024 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/dvfsrc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

/* DVFSRC_BASIC_CONTROL */
#define DVFSRC_V4_BASIC_CTRL_OPP_COUNT	GENMASK(26, 20)

/* DVFSRC_LEVEL */
#define DVFSRC_V1_LEVEL_TARGET_LEVEL	GENMASK(15, 0)
#define DVFSRC_TGT_LEVEL_IDLE		0x00
#define DVFSRC_V1_LEVEL_CURRENT_LEVEL	GENMASK(31, 16)

#define DVFSRC_V4_LEVEL_TARGET_LEVEL	GENMASK(15, 8)
#define DVFSRC_V4_LEVEL_TARGET_PRESENT	BIT(16)

/* DVFSRC_SW_REQ, DVFSRC_SW_REQ2 */
#define DVFSRC_V1_SW_REQ2_DRAM_LEVEL	GENMASK(1, 0)
#define DVFSRC_V1_SW_REQ2_VCORE_LEVEL	GENMASK(3, 2)

#define DVFSRC_V2_SW_REQ_DRAM_LEVEL	GENMASK(3, 0)
#define DVFSRC_V2_SW_REQ_VCORE_LEVEL	GENMASK(6, 4)

#define DVFSRC_V4_SW_REQ_EMI_LEVEL	GENMASK(3, 0)
#define DVFSRC_V4_SW_REQ_DRAM_LEVEL	GENMASK(15, 12)

/* DVFSRC_VCORE */
#define DVFSRC_V1_VCORE_REQ_VSCP_LEVEL	GENMASK(30, 3)
#define DVFSRC_V2_VCORE_REQ_VSCP_LEVEL	GENMASK(14, 12)

/* DVFSRC_TARGET_GEAR */
#define DVFSRC_V4_GEAR_TARGET_DRAM	GENMASK(7, 0)
#define DVFSRC_V4_GEAR_TARGET_VCORE	GENMASK(15, 8)

/* DVFSRC_GEAR_INFO */
#define DVFSRC_V4_GEAR_INFO_REG_WIDTH	0x4
#define DVFSRC_V4_GEAR_INFO_REG_LEVELS	64
#define DVFSRC_V4_GEAR_INFO_VCORE	GENMASK(3, 0)
#define DVFSRC_V4_GEAR_INFO_EMI		GENMASK(7, 4)
#define DVFSRC_V4_GEAR_INFO_DRAM	GENMASK(15, 12)

#define DVFSRC_POLL_TIMEOUT_US		1000
#define STARTUP_TIME_US			1

#define MTK_SIP_DVFSRC_INIT		0x0
#define MTK_SIP_DVFSRC_START		0x1

enum mtk_dvfsrc_bw_type {
	DVFSRC_BW_AVG,
	DVFSRC_BW_PEAK,
	DVFSRC_BW_HRT,
	DVFSRC_BW_MAX,
};

struct dvfsrc_opp {
	u32 vcore_opp;
	u32 dram_opp;
	u32 emi_opp;
};

struct dvfsrc_opp_desc {
	const struct dvfsrc_opp *opps;
	u32 num_opp;
};

struct dvfsrc_soc_data;
struct mtk_dvfsrc {
	struct device *dev;
	struct clk *clk;
	struct platform_device *icc;
	struct platform_device *regulator;
	const struct dvfsrc_soc_data *dvd;
	const struct dvfsrc_opp_desc *curr_opps;
	void __iomem *regs;
	int dram_type;
};

struct dvfsrc_soc_data {
	const int *regs;
	const u8 *bw_units;
	const bool has_emi_ddr;
	const struct dvfsrc_opp_desc *opps_desc;
	u32 (*calc_dram_bw)(struct mtk_dvfsrc *dvfsrc, enum mtk_dvfsrc_bw_type type, u64 bw);
	u32 (*get_target_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_current_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_vcore_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_vscp_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_opp_count)(struct mtk_dvfsrc *dvfsrc);
	int (*get_hw_opps)(struct mtk_dvfsrc *dvfsrc);
	void (*set_dram_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_dram_peak_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_dram_hrt_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_opp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	void (*set_vcore_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	void (*set_vscp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	int (*wait_for_opp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	int (*wait_for_vcore_level)(struct mtk_dvfsrc *dvfsrc, u32 level);

	/**
	 * @bw_max_constraints - array of maximum bandwidth for this hardware
	 *
	 * indexed by &enum mtk_dvfsrc_bw_type, storing the maximum permissible
	 * hardware value for each bandwidth type.
	 */
	const u32 *const bw_max_constraints;

	/**
	 * @bw_min_constraints - array of minimum bandwidth for this hardware
	 *
	 * indexed by &enum mtk_dvfsrc_bw_type, storing the minimum permissible
	 * hardware value for each bandwidth type.
	 */
	const u32 *const bw_min_constraints;
};

static u32 dvfsrc_readl(struct mtk_dvfsrc *dvfs, u32 offset)
{
	return readl(dvfs->regs + dvfs->dvd->regs[offset]);
}

static void dvfsrc_writel(struct mtk_dvfsrc *dvfs, u32 offset, u32 val)
{
	writel(val, dvfs->regs + dvfs->dvd->regs[offset]);
}

enum dvfsrc_regs {
	DVFSRC_BASIC_CONTROL,
	DVFSRC_SW_REQ,
	DVFSRC_SW_REQ2,
	DVFSRC_LEVEL,
	DVFSRC_TARGET_LEVEL,
	DVFSRC_SW_BW,
	DVFSRC_SW_PEAK_BW,
	DVFSRC_SW_HRT_BW,
	DVFSRC_SW_EMI_BW,
	DVFSRC_VCORE,
	DVFSRC_TARGET_GEAR,
	DVFSRC_GEAR_INFO_L,
	DVFSRC_GEAR_INFO_H,
	DVFSRC_REGS_MAX,
};

static const int dvfsrc_mt6768_regs[] = {
	[DVFSRC_SW_REQ] =		0x4,
	[DVFSRC_LEVEL] =		0xDC,
	[DVFSRC_SW_BW] =		0x16C,
	[DVFSRC_SW_PEAK_BW] =		0x160,
	[DVFSRC_VCORE] =	0x48,
};

static const int dvfsrc_mt8183_regs[] = {
	[DVFSRC_SW_REQ] = 0x4,
	[DVFSRC_SW_REQ2] = 0x8,
	[DVFSRC_LEVEL] = 0xDC,
	[DVFSRC_SW_BW] = 0x160,
};

static const int dvfsrc_mt8195_regs[] = {
	[DVFSRC_SW_REQ] = 0xc,
	[DVFSRC_VCORE] = 0x6c,
	[DVFSRC_SW_PEAK_BW] = 0x278,
	[DVFSRC_SW_BW] = 0x26c,
	[DVFSRC_SW_HRT_BW] = 0x290,
	[DVFSRC_LEVEL] = 0xd44,
	[DVFSRC_TARGET_LEVEL] = 0xd48,
};

static const int dvfsrc_mt8196_regs[] = {
	[DVFSRC_BASIC_CONTROL] = 0x0,
	[DVFSRC_SW_REQ] = 0x18,
	[DVFSRC_VCORE] = 0x80,
	[DVFSRC_GEAR_INFO_L] = 0xfc,
	[DVFSRC_SW_BW] = 0x1e8,
	[DVFSRC_SW_PEAK_BW] = 0x1f4,
	[DVFSRC_SW_HRT_BW] = 0x20c,
	[DVFSRC_LEVEL] = 0x5f0,
	[DVFSRC_TARGET_LEVEL] = 0x5f0,
	[DVFSRC_SW_REQ2] = 0x604,
	[DVFSRC_SW_EMI_BW] = 0x60c,
	[DVFSRC_TARGET_GEAR] = 0x6ac,
	[DVFSRC_GEAR_INFO_H] = 0x6b0,
};

static const struct dvfsrc_opp *dvfsrc_get_current_opp(struct mtk_dvfsrc *dvfsrc)
{
	u32 level = dvfsrc->dvd->get_current_level(dvfsrc);

	return &dvfsrc->curr_opps->opps[level];
}

static u32 dvfsrc_get_current_target_vcore_gear(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_TARGET_GEAR);

	return FIELD_GET(DVFSRC_V4_GEAR_TARGET_VCORE, val);
}

static u32 dvfsrc_get_current_target_dram_gear(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_TARGET_GEAR);

	return FIELD_GET(DVFSRC_V4_GEAR_TARGET_DRAM, val);
}

static bool dvfsrc_is_idle(struct mtk_dvfsrc *dvfsrc)
{
	if (!dvfsrc->dvd->get_target_level)
		return true;

	return dvfsrc->dvd->get_target_level(dvfsrc) == DVFSRC_TGT_LEVEL_IDLE;
}

static int dvfsrc_wait_for_vcore_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *curr;

	return readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					 curr->vcore_opp >= level, STARTUP_TIME_US,
					 DVFSRC_POLL_TIMEOUT_US);
}

static int dvfsrc_wait_for_opp_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *target, *curr;
	int ret;

	target = &dvfsrc->curr_opps->opps[level];
	ret = readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					curr->dram_opp >= target->dram_opp &&
					curr->vcore_opp >= target->vcore_opp,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "timeout! target OPP: %u, dram: %d, vcore: %d\n", level,
			 curr->dram_opp, curr->vcore_opp);
		return ret;
	}

	return 0;
}

static int dvfsrc_wait_for_opp_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *target, *curr;
	int ret;

	target = &dvfsrc->curr_opps->opps[level];
	ret = readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					curr->dram_opp >= target->dram_opp &&
					curr->vcore_opp >= target->vcore_opp,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "timeout! target OPP: %u, dram: %d\n", level, curr->dram_opp);
		return ret;
	}

	return 0;
}

static int dvfsrc_wait_for_vcore_level_v4(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val;

	return readx_poll_timeout_atomic(dvfsrc_get_current_target_vcore_gear,
					 dvfsrc, val, val >= level,
					 STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
}

static int dvfsrc_wait_for_opp_level_v4(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val;

	return readx_poll_timeout_atomic(dvfsrc_get_current_target_dram_gear,
					 dvfsrc, val, val >= level,
					 STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
}

static u32 dvfsrc_get_target_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);

	return FIELD_GET(DVFSRC_V1_LEVEL_TARGET_LEVEL, val);
}

static u32 dvfsrc_get_current_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);
	u32 current_level = FIELD_GET(DVFSRC_V1_LEVEL_CURRENT_LEVEL, val);

	return ffs(current_level) - 1;
}

static u32 dvfsrc_get_target_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	return dvfsrc_readl(dvfsrc, DVFSRC_TARGET_LEVEL);
}

static u32 dvfsrc_get_current_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);
	u32 level = ffs(val);

	/* Valid levels */
	if (level < dvfsrc->curr_opps->num_opp)
		return dvfsrc->curr_opps->num_opp - level;

	/* Zero for level 0 or invalid level */
	return 0;
}

static u32 dvfsrc_get_target_level_v4(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_TARGET_LEVEL);

	if (val & DVFSRC_V4_LEVEL_TARGET_PRESENT)
		return FIELD_GET(DVFSRC_V4_LEVEL_TARGET_LEVEL, val) + 1;
	return 0;
}

static u32 dvfsrc_get_current_level_v4(struct mtk_dvfsrc *dvfsrc)
{
	u32 level = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL) + 1;

	/* Valid levels */
	if (level < dvfsrc->curr_opps->num_opp)
		return dvfsrc->curr_opps->num_opp - level;

	/* Zero for level 0 or invalid level */
	return 0;
}

static u32 dvfsrc_get_vcore_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ2);

	return FIELD_GET(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, val);
}

static void dvfsrc_set_vcore_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ2);

	val &= ~DVFSRC_V1_SW_REQ2_VCORE_LEVEL;
	val |= FIELD_PREP(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ2, val);
}

static u32 dvfsrc_get_vcore_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	return FIELD_GET(DVFSRC_V2_SW_REQ_VCORE_LEVEL, val);
}

static void dvfsrc_set_vcore_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	val &= ~DVFSRC_V2_SW_REQ_VCORE_LEVEL;
	val |= FIELD_PREP(DVFSRC_V2_SW_REQ_VCORE_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
}

static u32 dvfsrc_get_vscp_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	return FIELD_GET(DVFSRC_V1_VCORE_REQ_VSCP_LEVEL, val);
}

static void dvfsrc_set_vscp_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	val &= ~DVFSRC_V1_VCORE_REQ_VSCP_LEVEL;
	val |= FIELD_PREP(DVFSRC_V1_VCORE_REQ_VSCP_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_VCORE, val);
}

static u32 dvfsrc_get_vscp_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	return FIELD_GET(DVFSRC_V2_VCORE_REQ_VSCP_LEVEL, val);
}

static void dvfsrc_set_vscp_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	val &= ~DVFSRC_V2_VCORE_REQ_VSCP_LEVEL;
	val |= FIELD_PREP(DVFSRC_V2_VCORE_REQ_VSCP_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_VCORE, val);
}

static u32 dvfsrc_get_opp_count_v4(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_BASIC_CONTROL);

	return FIELD_GET(DVFSRC_V4_BASIC_CTRL_OPP_COUNT, val) + 1;
}

static u32
dvfsrc_calc_dram_bw_v1(struct mtk_dvfsrc *dvfsrc, enum mtk_dvfsrc_bw_type type, u64 bw)
{
	return clamp_val(div_u64(bw, 100 * 1000), dvfsrc->dvd->bw_min_constraints[type],
			 dvfsrc->dvd->bw_max_constraints[type]);
}

/**
 * dvfsrc_calc_dram_bw_v4 - convert kbps to hardware register bandwidth value
 * @dvfsrc: pointer to the &struct mtk_dvfsrc of this driver instance
 * @type: one of %DVFSRC_BW_AVG, %DVFSRC_BW_PEAK, or %DVFSRC_BW_HRT
 * @bw: the bandwidth in kilobits per second
 *
 * Returns the hardware register value appropriate for expressing @bw, clamped
 * to hardware limits.
 */
static u32
dvfsrc_calc_dram_bw_v4(struct mtk_dvfsrc *dvfsrc, enum mtk_dvfsrc_bw_type type, u64 bw)
{
	u8 bw_unit = dvfsrc->dvd->bw_units[type];
	u64 bw_mbps;
	u32 bw_hw;

	if (type < DVFSRC_BW_AVG || type >= DVFSRC_BW_MAX)
		return 0;

	bw_mbps = div_u64(bw, 1000);
	bw_hw = div_u64((bw_mbps + bw_unit - 1), bw_unit);
	return clamp_val(bw_hw, dvfsrc->dvd->bw_min_constraints[type],
			 dvfsrc->dvd->bw_max_constraints[type]);
}

static void __dvfsrc_set_dram_bw_v1(struct mtk_dvfsrc *dvfsrc, u32 reg,
				    enum mtk_dvfsrc_bw_type type, u64 bw)
{
	u32 bw_hw = dvfsrc->dvd->calc_dram_bw(dvfsrc, type, bw);

	dvfsrc_writel(dvfsrc, reg, bw_hw);

	if (type == DVFSRC_BW_AVG && dvfsrc->dvd->has_emi_ddr)
		dvfsrc_writel(dvfsrc, DVFSRC_SW_EMI_BW, bw_hw);
}

static void dvfsrc_set_dram_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_BW, DVFSRC_BW_AVG, bw);
};

static void dvfsrc_set_dram_peak_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_PEAK_BW, DVFSRC_BW_PEAK, bw);
}

static void dvfsrc_set_dram_hrt_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_HRT_BW, DVFSRC_BW_HRT, bw);
}

static void dvfsrc_set_opp_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *opp = &dvfsrc->curr_opps->opps[level];
	u32 val;

	/* Translate Pstate to DVFSRC level and set it to DVFSRC HW */
	val = FIELD_PREP(DVFSRC_V1_SW_REQ2_DRAM_LEVEL, opp->dram_opp);
	val |= FIELD_PREP(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, opp->vcore_opp);

	dev_dbg(dvfsrc->dev, "vcore_opp: %d, dram_opp: %d\n", opp->vcore_opp, opp->dram_opp);
	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
}

static u32 dvfsrc_get_opp_gear(struct mtk_dvfsrc *dvfsrc, u8 level)
{
	u32 reg_ofst, val;
	u8 idx;

	/* Calculate register offset and index for requested gear */
	if (level < DVFSRC_V4_GEAR_INFO_REG_LEVELS) {
		reg_ofst = dvfsrc->dvd->regs[DVFSRC_GEAR_INFO_L];
		idx = level;
	} else {
		reg_ofst = dvfsrc->dvd->regs[DVFSRC_GEAR_INFO_H];
		idx = level - DVFSRC_V4_GEAR_INFO_REG_LEVELS;
	}
	reg_ofst += DVFSRC_V4_GEAR_INFO_REG_WIDTH * (level / 2);

	/* Read the corresponding gear register */
	val = readl(dvfsrc->regs + reg_ofst);

	/* Each register contains two sets of data, 16 bits per gear */
	val >>= 16 * (idx % 2);

	return val;
}

static int dvfsrc_get_hw_opps_v4(struct mtk_dvfsrc *dvfsrc)
{
	struct dvfsrc_opp *dvfsrc_opps;
	struct dvfsrc_opp_desc *desc;
	u32 num_opps, gear_info;
	u8 num_vcore, num_dram;
	u8 num_emi;
	int i;

	num_opps = dvfsrc_get_opp_count_v4(dvfsrc);
	if (num_opps == 0) {
		dev_err(dvfsrc->dev, "No OPPs programmed in DVFSRC MCU.\n");
		return -EINVAL;
	}

	/*
	 * The first 16 bits set in the gear info table says how many OPPs
	 * and how many vcore, dram and emi table entries are available.
	 */
	gear_info = dvfsrc_readl(dvfsrc, DVFSRC_GEAR_INFO_L);
	if (gear_info == 0) {
		dev_err(dvfsrc->dev, "No gear info in DVFSRC MCU.\n");
		return -EINVAL;
	}

	num_vcore = FIELD_GET(DVFSRC_V4_GEAR_INFO_VCORE, gear_info) + 1;
	num_dram = FIELD_GET(DVFSRC_V4_GEAR_INFO_DRAM, gear_info) + 1;
	num_emi = FIELD_GET(DVFSRC_V4_GEAR_INFO_EMI, gear_info) + 1;
	dev_info(dvfsrc->dev,
		 "Discovered %u gears and %u vcore, %u dram, %u emi table entries.\n",
		 num_opps, num_vcore, num_dram, num_emi);

	/* Allocate everything now as anything else after that cannot fail */
	desc = devm_kzalloc(dvfsrc->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	dvfsrc_opps = devm_kcalloc(dvfsrc->dev, num_opps + 1,
				   sizeof(*dvfsrc_opps), GFP_KERNEL);
	if (!dvfsrc_opps)
		return -ENOMEM;

	/* Read the OPP table gear indices */
	for (i = 0; i <= num_opps; i++) {
		gear_info = dvfsrc_get_opp_gear(dvfsrc, num_opps - i);
		dvfsrc_opps[i].vcore_opp = FIELD_GET(DVFSRC_V4_GEAR_INFO_VCORE, gear_info);
		dvfsrc_opps[i].dram_opp = FIELD_GET(DVFSRC_V4_GEAR_INFO_DRAM, gear_info);
		dvfsrc_opps[i].emi_opp = FIELD_GET(DVFSRC_V4_GEAR_INFO_EMI, gear_info);
	};
	desc->num_opp = num_opps + 1;
	desc->opps = dvfsrc_opps;

	/* Assign to main structure now that everything is done! */
	dvfsrc->curr_opps = desc;

	return 0;
}

static void dvfsrc_set_dram_level_v4(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	val &= ~DVFSRC_V4_SW_REQ_DRAM_LEVEL;
	val |= FIELD_PREP(DVFSRC_V4_SW_REQ_DRAM_LEVEL, level);

	dev_dbg(dvfsrc->dev, "%s level=%u\n", __func__, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
}

int mtk_dvfsrc_send_request(const struct device *dev, u32 cmd, u64 data)
{
	struct mtk_dvfsrc *dvfsrc = dev_get_drvdata(dev);
	bool state;
	int ret;

	dev_dbg(dvfsrc->dev, "cmd: %d, data: %llu\n", cmd, data);

	switch (cmd) {
	case MTK_DVFSRC_CMD_BW:
		dvfsrc->dvd->set_dram_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_HRT_BW:
		if (dvfsrc->dvd->set_dram_hrt_bw)
			dvfsrc->dvd->set_dram_hrt_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_PEAK_BW:
		if (dvfsrc->dvd->set_dram_peak_bw)
			dvfsrc->dvd->set_dram_peak_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_OPP:
		if (!dvfsrc->dvd->set_opp_level)
			return 0;

		dvfsrc->dvd->set_opp_level(dvfsrc, data);
		break;
	case MTK_DVFSRC_CMD_VCORE_LEVEL:
		dvfsrc->dvd->set_vcore_level(dvfsrc, data);
		break;
	case MTK_DVFSRC_CMD_VSCP_LEVEL:
		if (!dvfsrc->dvd->set_vscp_level)
			return 0;

		dvfsrc->dvd->set_vscp_level(dvfsrc, data);
		break;
	default:
		dev_err(dvfsrc->dev, "unknown command: %d\n", cmd);
		return -EOPNOTSUPP;
	}

	/* DVFSRC needs at least 2T(~196ns) to handle a request */
	udelay(STARTUP_TIME_US);

	ret = readx_poll_timeout_atomic(dvfsrc_is_idle, dvfsrc, state, state,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "%d: idle timeout, data: %llu, last: %d -> %d\n", cmd, data,
			 dvfsrc->dvd->get_current_level(dvfsrc),
			 dvfsrc->dvd->get_target_level(dvfsrc));
		return ret;
	}

	if (cmd == MTK_DVFSRC_CMD_OPP)
		ret = dvfsrc->dvd->wait_for_opp_level(dvfsrc, data);
	else
		ret = dvfsrc->dvd->wait_for_vcore_level(dvfsrc, data);

	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "%d: wait timeout, data: %llu, last: %d -> %d\n",
			 cmd, data,
			 dvfsrc->dvd->get_current_level(dvfsrc),
			 dvfsrc->dvd->get_target_level(dvfsrc));
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_dvfsrc_send_request);

int mtk_dvfsrc_query_info(const struct device *dev, u32 cmd, int *data)
{
	struct mtk_dvfsrc *dvfsrc = dev_get_drvdata(dev);

	switch (cmd) {
	case MTK_DVFSRC_CMD_VCORE_LEVEL:
		*data = dvfsrc->dvd->get_vcore_level(dvfsrc);
		break;
	case MTK_DVFSRC_CMD_VSCP_LEVEL:
		*data = dvfsrc->dvd->get_vscp_level(dvfsrc);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_dvfsrc_query_info);

#define DVFSRC_BASIC_CONTROL       (0x0)
#define DVFSRC_SW_REQ              (0x4)
#define DVFSRC_SW_REQ2             (0x8)
#define DVFSRC_EMI_REQUEST         (0xC)
#define DVFSRC_EMI_REQUEST2        (0x10)
#define DVFSRC_EMI_REQUEST3        (0x14)
#define DVFSRC_EMI_HRT             (0x18)
#define DVFSRC_EMI_HRT2            (0x1C)
#define DVFSRC_EMI_HRT3            (0x20)
#define DVFSRC_EMI_QOS0            (0x24)
#define DVFSRC_EMI_QOS1            (0x28)
#define DVFSRC_EMI_QOS2            (0x2C)
#define DVFSRC_EMI_MD2SPM0         (0x30)
#define DVFSRC_EMI_MD2SPM1         (0x34)
#define DVFSRC_EMI_MD2SPM2         (0x38)
#define DVFSRC_EMI_MD2SPM0_T       (0x3C)
#define DVFSRC_EMI_MD2SPM1_T       (0x40)
#define DVFSRC_EMI_MD2SPM2_T       (0x44)
#define DVFSRC_VCORE_REQUEST       (0x48)
#define DVFSRC_VCORE_REQUEST2      (0x4C)
#define DVFSRC_VCORE_HRT           (0x50)
#define DVFSRC_VCORE_HRT2          (0x54)
#define DVFSRC_VCORE_HRT3          (0x58)
#define DVFSRC_VCORE_QOS0          (0x5C)
#define DVFSRC_VCORE_QOS1          (0x60)
#define DVFSRC_VCORE_QOS2          (0x64)
#define DVFSRC_VCORE_MD2SPM0       (0x68)
#define DVFSRC_VCORE_MD2SPM1       (0x6C)
#define DVFSRC_VCORE_MD2SPM2       (0x70)
#define DVFSRC_VCORE_MD2SPM0_T     (0x74)
#define DVFSRC_VCORE_MD2SPM1_T     (0x78)
#define DVFSRC_VCORE_MD2SPM2_T     (0x7C)
#define DVFSRC_MD_REQUEST          (0x80)
#define DVFSRC_MD_SW_CONTROL	   (0x84)
#define DVFSRC_MD_VMODEM_REMAP     (0x88)
#define DVFSRC_MD_VMD_REMAP        (0x8C)
#define DVFSRC_MD_VSRAM_REMAP      (0x90)
#define DVFSRC_HALT_SW_CONTROL	   (0x94)
#define DVFSRC_INT                 (0x98)
#define DVFSRC_INT_EN              (0x9C)
#define DVFSRC_INT_CLR             (0xA0)
#define DVFSRC_BW_MON_WINDOW       (0xA4)
#define DVFSRC_BW_MON_THRES_1      (0xA8)
#define DVFSRC_BW_MON_THRES_2      (0xAC)
#define DVFSRC_MD_TURBO            (0xB0)
#define DVFSRC_DEBOUNCE_FOUR       (0xD0)
#define DVFSRC_DEBOUNCE_RISE_FALL  (0xD4)
#define DVFSRC_TIMEOUT_NEXTREQ     (0xD8)
#define DVFSRC_LEVEL               (0xDC)
#define DVFSRC_LEVEL_LABEL_0_1     (0xE0)
#define DVFSRC_LEVEL_LABEL_2_3     (0xE4)
#define DVFSRC_LEVEL_LABEL_4_5     (0xE8)
#define DVFSRC_LEVEL_LABEL_6_7     (0xEC)
#define DVFSRC_LEVEL_LABEL_8_9     (0xF0)
#define DVFSRC_LEVEL_LABEL_10_11   (0xF4)
#define DVFSRC_LEVEL_LABEL_12_13   (0xF8)
#define DVFSRC_LEVEL_LABEL_14_15   (0xFC)
#define DVFSRC_MM_BW_0             (0x100)
#define DVFSRC_MM_BW_1             (0x104)
#define DVFSRC_MM_BW_2             (0x108)
#define DVFSRC_MM_BW_3             (0x10C)
#define DVFSRC_MM_BW_4             (0x110)
#define DVFSRC_MM_BW_5             (0x114)
#define DVFSRC_MM_BW_6             (0x118)
#define DVFSRC_MM_BW_7             (0x11C)
#define DVFSRC_MM_BW_8             (0x120)
#define DVFSRC_MM_BW_9             (0x124)
#define DVFSRC_MM_BW_10            (0x128)
#define DVFSRC_MM_BW_11            (0x12C)
#define DVFSRC_MM_BW_12            (0x130)
#define DVFSRC_MM_BW_13            (0x134)
#define DVFSRC_MM_BW_14            (0x138)
#define DVFSRC_MM_BW_15            (0x13C)
#define DVFSRC_MD_BW_0             (0x140)
#define DVFSRC_MD_BW_1             (0x144)
#define DVFSRC_MD_BW_2             (0x148)
#define DVFSRC_MD_BW_3             (0x14C)
#define DVFSRC_MD_BW_4             (0x150)
#define DVFSRC_MD_BW_5             (0x154)
#define DVFSRC_MD_BW_6             (0x158)
#define DVFSRC_MD_BW_7             (0x15C)
#define DVFSRC_SW_BW_0             (0x160)
#define DVFSRC_SW_BW_1             (0x164)
#define DVFSRC_SW_BW_2             (0x168)
#define DVFSRC_SW_BW_3             (0x16C)
#define DVFSRC_SW_BW_4             (0x170)
#define DVFSRC_QOS_EN              (0x180)
#define DVFSRC_ISP_HRT             (0x190)
#define DVFSRC_FORCE               (0x300)
#define DVFSRC_SEC_SW_REQ          (0x304)
#define DVFSRC_LAST                (0x308)
#define DVFSRC_LAST_L              (0x30C)
#define DVFSRC_MD_SCENARIO         (0X310)
#define DVFSRC_RECORD_0_0          (0x400)
#define DVFSRC_RECORD_0_1          (0x404)
#define DVFSRC_RECORD_0_2          (0x408)
#define DVFSRC_RECORD_1_0          (0x40C)
#define DVFSRC_RECORD_1_1          (0x410)
#define DVFSRC_RECORD_1_2          (0x414)
#define DVFSRC_RECORD_2_0          (0x418)
#define DVFSRC_RECORD_2_1          (0x41C)
#define DVFSRC_RECORD_2_2          (0x420)
#define DVFSRC_RECORD_3_0          (0x424)
#define DVFSRC_RECORD_3_1          (0x428)
#define DVFSRC_RECORD_3_2          (0x42C)
#define DVFSRC_RECORD_4_0          (0x430)
#define DVFSRC_RECORD_4_1          (0x434)
#define DVFSRC_RECORD_4_2          (0x438)
#define DVFSRC_RECORD_5_0          (0x43C)
#define DVFSRC_RECORD_5_1          (0x440)
#define DVFSRC_RECORD_5_2          (0x444)
#define DVFSRC_RECORD_6_0          (0x448)
#define DVFSRC_RECORD_6_1          (0x44C)
#define DVFSRC_RECORD_6_2          (0x450)
#define DVFSRC_RECORD_7_0          (0x454)
#define DVFSRC_RECORD_7_1          (0x458)
#define DVFSRC_RECORD_7_2          (0x45C)
#define DVFSRC_RECORD_0_L_0        (0x460)
#define DVFSRC_RECORD_0_L_1        (0x464)
#define DVFSRC_RECORD_0_L_2        (0x468)
#define DVFSRC_RECORD_1_L_0        (0x46C)
#define DVFSRC_RECORD_1_L_1        (0x470)
#define DVFSRC_RECORD_1_L_2        (0x474)
#define DVFSRC_RECORD_2_L_0        (0x478)
#define DVFSRC_RECORD_2_L_1        (0x47C)
#define DVFSRC_RECORD_2_L_2        (0x480)
#define DVFSRC_RECORD_3_L_0        (0x484)
#define DVFSRC_RECORD_3_L_1        (0x488)
#define DVFSRC_RECORD_3_L_2        (0x48C)
#define DVFSRC_RECORD_4_L_0        (0x490)
#define DVFSRC_RECORD_4_L_1        (0x494)
#define DVFSRC_RECORD_4_L_2        (0x498)
#define DVFSRC_RECORD_5_L_0        (0x49C)
#define DVFSRC_RECORD_5_L_1        (0x4A0)
#define DVFSRC_RECORD_5_L_2        (0x4A4)
#define DVFSRC_RECORD_6_L_0        (0x4A8)
#define DVFSRC_RECORD_6_L_1        (0x4AC)
#define DVFSRC_RECORD_6_L_2        (0x4B0)
#define DVFSRC_RECORD_7_L_0        (0x4B4)
#define DVFSRC_RECORD_7_L_1        (0x4B8)
#define DVFSRC_RECORD_7_L_2        (0x4BC)
#define DVFSRC_RECORD_MD_0         (0x4C0)
#define DVFSRC_RECORD_MD_1         (0x4C4)
#define DVFSRC_RECORD_MD_2         (0x4C8)
#define DVFSRC_RECORD_MD_3         (0x4CC)
#define DVFSRC_RECORD_MD_4         (0x4D0)
#define DVFSRC_RECORD_MD_5         (0x4D4)
#define DVFSRC_RECORD_MD_6         (0x4D8)
#define DVFSRC_RECORD_MD_7         (0x4DC)
#define DVFSRC_RECORD_COUNT        (0x4F0)
#define DVFSRC_RSRV_0              (0x600)
#define DVFSRC_RSRV_1              (0x604)
#define DVFSRC_RSRV_2              (0x608)
#define DVFSRC_RSRV_3              (0x60C)
#define DVFSRC_RSRV_4              (0x610)
#define DVFSRC_RSRV_5              (0x614)

static int dvfsrc_lp4x_2ch_3600[][128] = {
		{ DVFSRC_EMI_REQUEST,		0x00240009 },
		{ DVFSRC_EMI_REQUEST3,		0x09000000 },
		{ DVFSRC_EMI_HRT,		0x003E362C },
		{ DVFSRC_EMI_QOS0,		0x00000033 },
		{ DVFSRC_EMI_QOS1,		0x0000004C },
		{ DVFSRC_EMI_MD2SPM0,		0x0000003F },
		{ DVFSRC_EMI_MD2SPM1,		0x00000000 },
		{ DVFSRC_EMI_MD2SPM2,		0x000080C0 },
		{ DVFSRC_EMI_MD2SPM0_T,		0x00000007 },
		{ DVFSRC_EMI_MD2SPM1_T,		0x00000038 },
		{ DVFSRC_EMI_MD2SPM2_T,		0x000080C0 },

		{ DVFSRC_VCORE_HRT,		0x00000036 },

		{ DVFSRC_MD_SW_CONTROL,		0x20000000 },

		{ DVFSRC_TIMEOUT_NEXTREQ,	0x00000014 },
		{ DVFSRC_INT_EN,		0x00000003 },

		{ DVFSRC_LEVEL_LABEL_0_1,	0x00010000 },
		{ DVFSRC_LEVEL_LABEL_2_3,	0x00020101 },
		{ DVFSRC_LEVEL_LABEL_4_5,	0x01020012 },
		{ DVFSRC_LEVEL_LABEL_6_7,	0x02120112 },
		{ DVFSRC_LEVEL_LABEL_8_9,	0x00230013 },
		{ DVFSRC_LEVEL_LABEL_10_11,	0x01230113 },
		{ DVFSRC_LEVEL_LABEL_12_13,	0x02230213 },
		{ DVFSRC_LEVEL_LABEL_14_15,	0x03230323 },

		{ DVFSRC_FORCE,			0x40000000 },
		{ DVFSRC_RSRV_1,		0x0000000C },

		{ DVFSRC_QOS_EN,		0x0000407F },

		{ DVFSRC_BASIC_CONTROL,		0x0000407B },
		{ DVFSRC_BASIC_CONTROL,		0x0000017B },
		{ DVFSRC_FORCE,			0x00000000 },

		{ -1, 0 },
};

static int mtk_dvfsrc_probe(struct platform_device *pdev)
{
	struct arm_smccc_res ares;
	struct mtk_dvfsrc *dvfsrc;
	struct device_node *np;
	struct regmap *regmap;
	int ret, val;

	dvfsrc = devm_kzalloc(&pdev->dev, sizeof(*dvfsrc), GFP_KERNEL);
	if (!dvfsrc)
		return -ENOMEM;

	dvfsrc->dvd = of_device_get_match_data(&pdev->dev);
	dvfsrc->dev = &pdev->dev;

	dvfsrc->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(dvfsrc->regs))
		return PTR_ERR(dvfsrc->regs);

	dvfsrc->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(dvfsrc->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(dvfsrc->clk),
				     "Couldn't get and enable DVFSRC clock\n");

	np = of_parse_phandle(pdev->dev.of_node, "mediatek,rgu", 0);

	if (!np) {
		arm_smccc_smc(MTK_SIP_DVFSRC_VCOREFS_CONTROL, MTK_SIP_DVFSRC_INIT,
		      0, 0, 0, 0, 0, 0, &ares);
		if (ares.a0)
			return dev_err_probe(&pdev->dev, -EINVAL, "DVFSRC init failed: %lu\n", ares.a0);

		dvfsrc->dram_type = ares.a1;
	} else {
		regmap = syscon_node_to_regmap(np);

		/* mtk_rgu_cfg_dvfsrc(1) */
		ret = regmap_read(regmap, 0xA0, &val);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "Failed to read MTK_WDT_DEBUG_CTL2\n");

		ret = regmap_write(regmap, 0xA0, val | BIT(9) | 0x55000000);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "Failed to write to MTK_WDT_DEBUG_CTL2\n");

		ret = regmap_read(regmap, 0x44, &val);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "Failed to read MTK_WDT_LATCH_CTL\n");

		ret = regmap_write(regmap, 0x44, val | BIT(13) | 0x95000000);
		if (ret)
			return dev_err_probe(&pdev->dev, ret, "Failed to write to MTK_WDT_LATCH_CTL\n");

		/* helio_dvfsrc_sram_reg_init() */
		for (val = 0; val < 0x80; val += 4)
			writel_relaxed(0, dvfsrc->regs + val);

		/* helio_dvfsrc_reg_config() */
		val = 0;
		while (dvfsrc_lp4x_2ch_3600[val][0] != 0) {
			writel(dvfsrc_lp4x_2ch_3600[val][1], dvfsrc->regs + dvfsrc_lp4x_2ch_3600[val][0]);
			val++;
		}

		dvfsrc->dram_type = 0;

		dev_info(&pdev->dev, "DVFSRC should be ready now...\n");
	}

	dev_dbg(&pdev->dev, "DRAM Type: %d\n", dvfsrc->dram_type);

	/* Newer versions of the DVFSRC MCU have pre-programmed gear tables */
	if (dvfsrc->dvd->get_hw_opps) {
		ret = dvfsrc->dvd->get_hw_opps(dvfsrc);
		if (ret)
			return ret;
	} else {
		dvfsrc->curr_opps = &dvfsrc->dvd->opps_desc[dvfsrc->dram_type];
	}
	platform_set_drvdata(pdev, dvfsrc);

	ret = devm_of_platform_populate(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to populate child devices\n");

	/* Everything is set up - make it run! */
	if (!np)
		arm_smccc_smc(MTK_SIP_DVFSRC_VCOREFS_CONTROL, MTK_SIP_DVFSRC_START,
		      0, 0, 0, 0, 0, 0, &ares);
	else
		/* helio_dvfsrc_enable(1) */
		arm_smccc_smc(0x82000226 | 0x40000000, BIT(3) | BIT(10), 0, 0, 0, 0, 0, 0, &ares); // MTK_SIP_KERNEL_SPM_VCOREFS_ARGS, SPM_FLAG_DIS_VCORE_DVS | SPM_FLAG_RUN_COMMON_SCENARIO
	if (ares.a0 & BIT(0))
		return dev_err_probe(&pdev->dev, -EINVAL, "Cannot start DVFSRC: %lu\n", ares.a0);

	return 0;
}

static const u32 dvfsrc_bw_min_constr_none[DVFSRC_BW_MAX] = {
	[DVFSRC_BW_AVG] = 0,
	[DVFSRC_BW_PEAK] = 0,
	[DVFSRC_BW_HRT] = 0,
};

static const u32 dvfsrc_bw_max_constr_v1[DVFSRC_BW_MAX] = {
	[DVFSRC_BW_AVG] = U32_MAX,
	[DVFSRC_BW_PEAK] = U32_MAX,
	[DVFSRC_BW_HRT] = U32_MAX,
};

static const u32 dvfsrc_bw_max_constr_v2[DVFSRC_BW_MAX] = {
	[DVFSRC_BW_AVG] = 65535,
	[DVFSRC_BW_PEAK] = 65535,
	[DVFSRC_BW_HRT] = 1023,
};

static const struct dvfsrc_opp dvfsrc_opp_mt6768[] = {
	{0, 0}, {1, 0}, {1, 0}, {2, 0},
	{2, 1}, {2, 0}, {2, 1}, {2, 1},
	{3, 1}, {3, 2}, {3, 1}, {3, 2},
	{3, 1}, {3, 2}, {3, 2}, {3, 2},
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt6768_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt6768,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt6768),
	}
};

static const struct dvfsrc_soc_data mt6768_data = {
	.opps_desc = dvfsrc_opp_mt6768_desc,
	.regs = dvfsrc_mt6768_regs,
	.calc_dram_bw = dvfsrc_calc_dram_bw_v1,
	.get_target_level = dvfsrc_get_target_level_v2,
	.get_current_level = dvfsrc_get_current_level_v2,
	.get_vcore_level = dvfsrc_get_vcore_level_v1,
	.get_vscp_level = dvfsrc_get_vscp_level_v1,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v1,
	.set_vscp_level = dvfsrc_set_vscp_level_v1,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v2,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_max_constraints = dvfsrc_bw_max_constr_v1,
	.bw_min_constraints = dvfsrc_bw_min_constr_none,
};


static const struct dvfsrc_opp dvfsrc_opp_mt6893_lp4[] = {
	{ 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 },
	{ 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 },
	{ 0, 2 }, { 1, 2 }, { 2, 2 }, { 3, 2 },
	{ 0, 3 }, { 1, 3 }, { 2, 3 }, { 3, 3 },
	{ 1, 4 }, { 2, 4 }, { 3, 4 }, { 2, 5 },
	{ 3, 5 }, { 3, 6 }, { 4, 6 }, { 4, 7 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt6893_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt6893_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt6893_lp4),
	}
};

static const struct dvfsrc_soc_data mt6893_data = {
	.opps_desc = dvfsrc_opp_mt6893_desc,
	.regs = dvfsrc_mt8195_regs,
	.get_target_level = dvfsrc_get_target_level_v2,
	.get_current_level = dvfsrc_get_current_level_v2,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v2,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_max_constraints = dvfsrc_bw_max_constr_v2,
	.bw_min_constraints = dvfsrc_bw_min_constr_none,
};

static const struct dvfsrc_opp dvfsrc_opp_mt8183_lp4[] = {
	{ 0, 0 }, { 0, 1 }, { 0, 2 }, { 1, 2 },
};

static const struct dvfsrc_opp dvfsrc_opp_mt8183_lp3[] = {
	{ 0, 0 }, { 0, 1 }, { 1, 1 }, { 1, 2 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt8183_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt8183_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp4),
	},
	[1] = {
		.opps = dvfsrc_opp_mt8183_lp3,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp3),
	},
	[2] = {
		.opps = dvfsrc_opp_mt8183_lp3,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp3),
	}
};

static const struct dvfsrc_soc_data mt8183_data = {
	.opps_desc = dvfsrc_opp_mt8183_desc,
	.regs = dvfsrc_mt8183_regs,
	.calc_dram_bw = dvfsrc_calc_dram_bw_v1,
	.get_target_level = dvfsrc_get_target_level_v1,
	.get_current_level = dvfsrc_get_current_level_v1,
	.get_vcore_level = dvfsrc_get_vcore_level_v1,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_opp_level = dvfsrc_set_opp_level_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v1,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v1,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_max_constraints = dvfsrc_bw_max_constr_v1,
	.bw_min_constraints = dvfsrc_bw_min_constr_none,
};

static const struct dvfsrc_opp dvfsrc_opp_mt8195_lp4[] = {
	{ 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 },
	{ 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 },
	{ 0, 2 }, { 1, 2 }, { 2, 2 }, { 3, 2 },
	{ 1, 3 }, { 2, 3 }, { 3, 3 }, { 1, 4 },
	{ 2, 4 }, { 3, 4 }, { 2, 5 }, { 3, 5 },
	{ 3, 6 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt8195_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt8195_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8195_lp4),
	}
};

static const struct dvfsrc_soc_data mt8195_data = {
	.opps_desc = dvfsrc_opp_mt8195_desc,
	.regs = dvfsrc_mt8195_regs,
	.calc_dram_bw = dvfsrc_calc_dram_bw_v1,
	.get_target_level = dvfsrc_get_target_level_v2,
	.get_current_level = dvfsrc_get_current_level_v2,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v2,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_max_constraints = dvfsrc_bw_max_constr_v2,
	.bw_min_constraints = dvfsrc_bw_min_constr_none,
};

static const u8 mt8196_bw_units[] = {
	[DVFSRC_BW_AVG] = 64,
	[DVFSRC_BW_PEAK] = 64,
	[DVFSRC_BW_HRT] = 30,
};

static const struct dvfsrc_soc_data mt8196_data = {
	.regs = dvfsrc_mt8196_regs,
	.bw_units = mt8196_bw_units,
	.has_emi_ddr = true,
	.get_target_level = dvfsrc_get_target_level_v4,
	.get_current_level = dvfsrc_get_current_level_v4,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.get_opp_count = dvfsrc_get_opp_count_v4,
	.get_hw_opps = dvfsrc_get_hw_opps_v4,
	.calc_dram_bw = dvfsrc_calc_dram_bw_v4,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_opp_level = dvfsrc_set_dram_level_v4,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v4,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v4,
	.bw_max_constraints = dvfsrc_bw_max_constr_v2,
	.bw_min_constraints = dvfsrc_bw_min_constr_none,
};

static const struct of_device_id mtk_dvfsrc_of_match[] = {
	{ .compatible = "mediatek,mt6768-dvfsrc", .data = &mt6768_data },
	{ .compatible = "mediatek,mt6893-dvfsrc", .data = &mt6893_data },
	{ .compatible = "mediatek,mt8183-dvfsrc", .data = &mt8183_data },
	{ .compatible = "mediatek,mt8195-dvfsrc", .data = &mt8195_data },
	{ .compatible = "mediatek,mt8196-dvfsrc", .data = &mt8196_data },
	{ /* sentinel */ }
};

static struct platform_driver mtk_dvfsrc_driver = {
	.probe	= mtk_dvfsrc_probe,
	.driver = {
		.name = "mtk-dvfsrc",
		.of_match_table = mtk_dvfsrc_of_match,
	},
};
module_platform_driver(mtk_dvfsrc_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_AUTHOR("Dawei Chien <dawei.chien@mediatek.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek DVFSRC driver");
