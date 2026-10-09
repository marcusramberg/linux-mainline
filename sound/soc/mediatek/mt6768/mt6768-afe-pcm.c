// SPDX-License-Identifier: GPL-2.0
//
// Mediatek ALSA SoC AFE platform driver for 6768
//
// Copyright (c) 2018 MediaTek Inc.
// Author: Michael Hsiao <michael.hsiao@mediatek.com>

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "mt6768-afe-common.h"
#include "mt6768-interconnection.h"
#include "../common/mtk-base-afe.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-afe-fe-dai.h"

/* memif/irq fs field; 3 and 7 are 384k and 352k, unused here */
enum {
	MTK_AFE_RATE_8K = 0,
	MTK_AFE_RATE_11K = 1,
	MTK_AFE_RATE_12K = 2,
	MTK_AFE_RATE_16K = 4,
	MTK_AFE_RATE_22K = 5,
	MTK_AFE_RATE_24K = 6,
	MTK_AFE_RATE_32K = 8,
	MTK_AFE_RATE_44K = 9,
	MTK_AFE_RATE_48K = 10,
	MTK_AFE_RATE_88K = 11,
	MTK_AFE_RATE_96K = 12,
	MTK_AFE_RATE_176K = 13,
	MTK_AFE_RATE_192K = 14,
};

unsigned int mt6768_general_rate_transform(struct device *dev,
					   unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_RATE_8K;
	case 11025:
		return MTK_AFE_RATE_11K;
	case 12000:
		return MTK_AFE_RATE_12K;
	case 16000:
		return MTK_AFE_RATE_16K;
	case 22050:
		return MTK_AFE_RATE_22K;
	case 24000:
		return MTK_AFE_RATE_24K;
	case 32000:
		return MTK_AFE_RATE_32K;
	case 44100:
		return MTK_AFE_RATE_44K;
	case 48000:
		return MTK_AFE_RATE_48K;
	case 88200:
		return MTK_AFE_RATE_88K;
	case 96000:
		return MTK_AFE_RATE_96K;
	case 176400:
		return MTK_AFE_RATE_176K;
	case 192000:
		return MTK_AFE_RATE_192K;
	default:
		dev_warn(dev, "%s(), rate %u invalid, use 48kHz\n",
			 __func__, rate);
		return MTK_AFE_RATE_48K;
	}
}

static int mt6768_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct mtk_base_afe *afe =
		snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));

	return mt6768_general_rate_transform(afe->dev, rate);
}

static int mt6768_irq_fs(struct snd_pcm_substream *substream,
			 unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct mtk_base_afe *afe =
		snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));

	return mt6768_general_rate_transform(afe->dev, rate);
}

static const struct snd_pcm_hardware mt6768_afe_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_S24_LE |
		    SNDRV_PCM_FMTBIT_S32_LE),
	.period_bytes_min = 256,
	.period_bytes_max = 4 * 48 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.buffer_bytes_max = 8 * 48 * 1024,
	.fifo_size = 0,
};

static const struct mtk_base_memif_data memif_data[MT6768_MEMIF_NUM] = {
	[MT6768_MEMIF_DL1] = {
		.name = "DL1",
		.id = MT6768_MEMIF_DL1,
		.reg_ofs_base = AFE_DL1_BASE,
		.reg_ofs_cur = AFE_DL1_CUR,
		.reg_ofs_end = AFE_DL1_END,
		.reg_ofs_base_msb = AFE_DL1_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL1_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL1_END_MSB,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = DL1_MODE_SFT,
		.fs_maskbit = DL1_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = DL1_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL1_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = DL1_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = DL1_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.pbuf_reg = AFE_MEMIF_PBUF_SIZE,
		.pbuf_mask = DL1_PBUF_SIZE_MASK,
		.pbuf_shift = DL1_PBUF_SIZE_SFT,
		.minlen_reg = AFE_MEMIF_MINLEN,
		.minlen_mask = DL1_MINLEN_MASK,
		.minlen_shift = DL1_MINLEN_SFT,
	},
	[MT6768_MEMIF_DL12] = {
		.name = "DL12",
		.id = MT6768_MEMIF_DL12,
		.reg_ofs_base = AFE_DL1_D2_BASE,
		.reg_ofs_cur = AFE_DL1_D2_CUR,
		.reg_ofs_end = AFE_DL1_D2_END,
		.reg_ofs_base_msb = AFE_DL1_D2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL1_D2_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL1_D2_END_MSB,
		.fs_reg = AFE_DAC_CON0,
		.fs_shift = DL1_DATA2_MODE_SFT,
		.fs_maskbit = DL1_DATA2_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = DL1_DATA2_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL12_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = DL12_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = DL12_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.pbuf_reg = AFE_MEMIF_PBUF_SIZE,
		.pbuf_mask = DL1_DATA2_PBUF_SIZE_MASK,
		.pbuf_shift = DL1_DATA2_PBUF_SIZE_SFT,
		.minlen_reg = AFE_MEMIF_MINLEN,
		.minlen_mask = DL1_DATA2_MINLEN_MASK,
		.minlen_shift = DL1_DATA2_MINLEN_SFT,
	},
	[MT6768_MEMIF_DL2] = {
		.name = "DL2",
		.id = MT6768_MEMIF_DL2,
		.reg_ofs_base = AFE_DL2_BASE,
		.reg_ofs_cur = AFE_DL2_CUR,
		.reg_ofs_end = AFE_DL2_END,
		.reg_ofs_base_msb = AFE_DL2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL2_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL2_END_MSB,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = DL2_MODE_SFT,
		.fs_maskbit = DL2_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = DL2_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL2_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = DL2_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = DL2_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.pbuf_reg = AFE_MEMIF_PBUF_SIZE,
		.pbuf_mask = DL2_PBUF_SIZE_MASK,
		.pbuf_shift = DL2_PBUF_SIZE_SFT,
		.minlen_reg = AFE_MEMIF_MINLEN,
		.minlen_mask = DL2_MINLEN_MASK,
		.minlen_shift = DL2_MINLEN_SFT,
	},
	[MT6768_MEMIF_DL3] = {
		.name = "DL3",
		.id = MT6768_MEMIF_DL3,
		.reg_ofs_base = AFE_DL3_BASE,
		.reg_ofs_cur = AFE_DL3_CUR,
		.reg_ofs_end = AFE_DL3_END,
		.reg_ofs_base_msb = AFE_DL3_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL3_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL3_END_MSB,
		.fs_reg = AFE_DAC_CON2,
		.fs_shift = DL3_MODE_SFT,
		.fs_maskbit = DL3_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = DL3_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL3_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = DL3_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = DL3_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.pbuf_reg = AFE_MEMIF_PBUF_SIZE,
		.pbuf_mask = DL3_PBUF_SIZE_MASK,
		.pbuf_shift = DL3_PBUF_SIZE_SFT,
		.minlen_reg = AFE_MEMIF_MINLEN,
		.minlen_mask = DL3_MINLEN_MASK,
		.minlen_shift = DL3_MINLEN_SFT,
	},
	[MT6768_MEMIF_VUL] = {
		.name = "VUL",
		.id = MT6768_MEMIF_VUL,
		.reg_ofs_base = AFE_VUL_BASE,
		.reg_ofs_cur = AFE_VUL_CUR,
		.reg_ofs_end = AFE_VUL_END,
		.reg_ofs_base_msb = AFE_VUL_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL_END_MSB,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = VUL_MODE_SFT,
		.fs_maskbit = VUL_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = VUL_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = VUL_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = VUL_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
	},
	[MT6768_MEMIF_VUL12] = {
		.name = "VUL12",
		.id = MT6768_MEMIF_VUL12,
		.reg_ofs_base = AFE_VUL_D2_BASE,
		.reg_ofs_cur = AFE_VUL_D2_CUR,
		.reg_ofs_end = AFE_VUL_D2_END,
		.reg_ofs_base_msb = AFE_VUL_D2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL_D2_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL_D2_END_MSB,
		.fs_reg = AFE_DAC_CON0,
		.fs_shift = VUL_DATA2_MODE_SFT,
		.fs_maskbit = VUL_DATA2_MODE_MASK,
		.mono_reg = AFE_DAC_CON0,
		.mono_shift = VUL_DATA2_DATA_SFT,
		.quad_ch_reg = AFE_MEMIF_PBUF_SIZE,
		.quad_ch_mask = VUL12_4CH_EN_MASK,
		.quad_ch_shift = VUL12_4CH_EN_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL_DATA2_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = VUL12_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = VUL12_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
	},
	[MT6768_MEMIF_VUL2] = {
		.name = "VUL2",
		.id = MT6768_MEMIF_VUL2,
		.reg_ofs_base = AFE_VUL2_BASE,
		.reg_ofs_cur = AFE_VUL2_CUR,
		.reg_ofs_end = AFE_VUL2_END,
		.reg_ofs_base_msb = AFE_VUL2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL2_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL2_END_MSB,
		.fs_reg = AFE_DAC_CON2,
		.fs_shift = VUL2_MODE_SFT,
		.fs_maskbit = VUL2_MODE_MASK,
		.mono_reg = AFE_DAC_CON2,
		.mono_shift = VUL2_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL2_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = VUL2_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = VUL2_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
	},
	[MT6768_MEMIF_AWB] = {
		.name = "AWB",
		.id = MT6768_MEMIF_AWB,
		.reg_ofs_base = AFE_AWB_BASE,
		.reg_ofs_cur = AFE_AWB_CUR,
		.reg_ofs_end = AFE_AWB_END,
		.reg_ofs_base_msb = AFE_AWB_BASE_MSB,
		.reg_ofs_cur_msb = AFE_AWB_CUR_MSB,
		.reg_ofs_end_msb = AFE_AWB_END_MSB,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = AWB_MODE_SFT,
		.fs_maskbit = AWB_MODE_MASK,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = AWB_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = AWB_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = AWB_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = AWB_HD_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
	},
	[MT6768_MEMIF_AWB2] = {
		.name = "AWB2",
		.id = MT6768_MEMIF_AWB2,
		.reg_ofs_base = AFE_AWB2_BASE,
		.reg_ofs_cur = AFE_AWB2_CUR,
		.reg_ofs_end = AFE_AWB2_END,
		.reg_ofs_base_msb = AFE_AWB2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_AWB2_CUR_MSB,
		.reg_ofs_end_msb = AFE_AWB2_END_MSB,
		.fs_reg = AFE_DAC_CON2,
		.fs_shift = AWB2_MODE_SFT,
		.fs_maskbit = AWB2_MODE_MASK,
		.mono_reg = AFE_DAC_CON2,
		.mono_shift = AWB2_DATA_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = AWB2_ON_SFT,
		.hd_reg = AFE_MEMIF_HD_MODE,
		.hd_shift = AWB2_HD_SFT,
		.hd_align_reg = AFE_MEMIF_HDALIGN,
		.hd_align_mshift = AWB2_ALIGN_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
	},
};

static const struct mtk_base_irq_data irq_data[MT6768_IRQ_NUM] = {
	[MT6768_IRQ_0] = {
		.id = MT6768_IRQ_0,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT0,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 0,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ0_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ0_MCU_CLR_SFT,
	},
	[MT6768_IRQ_1] = {
		.id = MT6768_IRQ_1,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT1,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 4,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ1_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ1_MCU_CLR_SFT,
	},
	[MT6768_IRQ_2] = {
		.id = MT6768_IRQ_2,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT2,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 8,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ2_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ2_MCU_CLR_SFT,
	},
	[MT6768_IRQ_3] = {
		.id = MT6768_IRQ_3,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT3,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 12,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ3_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ3_MCU_CLR_SFT,
	},
	[MT6768_IRQ_4] = {
		.id = MT6768_IRQ_4,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT4,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 16,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ4_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ4_MCU_CLR_SFT,
	},
	[MT6768_IRQ_5] = {
		.id = MT6768_IRQ_5,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT5,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 20,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ5_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ5_MCU_CLR_SFT,
	},
	[MT6768_IRQ_6] = {
		.id = MT6768_IRQ_6,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT6,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 24,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ6_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ6_MCU_CLR_SFT,
	},
	[MT6768_IRQ_7] = {
		.id = MT6768_IRQ_7,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT7,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = 28,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ7_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ7_MCU_CLR_SFT,
	},
	[MT6768_IRQ_11] = {
		.id = MT6768_IRQ_11,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT11,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = 0,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ11_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ11_MCU_CLR_SFT,
	},
	[MT6768_IRQ_12] = {
		.id = MT6768_IRQ_12,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT12,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = 4,
		.irq_fs_maskbit = 0xf,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ12_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ12_MCU_CLR_SFT,
	},
};

/* FE DAIs */
static struct snd_soc_dai_driver mt6768_memif_dai_driver[] = {
	{
		.name = "DL1",
		.id = MT6768_MEMIF_DL1,
		.playback = {
			.stream_name = "DL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "DL12",
		.id = MT6768_MEMIF_DL12,
		.playback = {
			.stream_name = "DL12",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "DL2",
		.id = MT6768_MEMIF_DL2,
		.playback = {
			.stream_name = "DL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "DL3",
		.id = MT6768_MEMIF_DL3,
		.playback = {
			.stream_name = "DL3",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "UL1",
		.id = MT6768_MEMIF_VUL12,
		.capture = {
			.stream_name = "UL1",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "UL2",
		.id = MT6768_MEMIF_AWB,
		.capture = {
			.stream_name = "UL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "UL3",
		.id = MT6768_MEMIF_VUL2,
		.capture = {
			.stream_name = "UL3",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "UL4",
		.id = MT6768_MEMIF_AWB2,
		.capture = {
			.stream_name = "UL4",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
		.name = "UL7",
		.id = MT6768_MEMIF_VUL,
		.capture = {
			.stream_name = "UL7",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	},
};

static const struct snd_kcontrol_new memif_ul1_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN21,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN22,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch3_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN9,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch4_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN10,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul7_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN9,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul7_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN10,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_soc_dapm_widget mt6768_memif_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("UL1_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch1_mix, ARRAY_SIZE(memif_ul1_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch2_mix, ARRAY_SIZE(memif_ul1_ch2_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH3", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch3_mix, ARRAY_SIZE(memif_ul1_ch3_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH4", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch4_mix, ARRAY_SIZE(memif_ul1_ch4_mix)),
	SND_SOC_DAPM_MIXER("UL7_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul7_ch1_mix, ARRAY_SIZE(memif_ul7_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL7_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul7_ch2_mix, ARRAY_SIZE(memif_ul7_ch2_mix)),
};

static const struct snd_soc_dapm_route mt6768_memif_routes[] = {
	/* capture */
	{"UL1", NULL, "UL1_CH1"},
	{"UL1", NULL, "UL1_CH2"},
	{"UL1", NULL, "UL1_CH3"},
	{"UL1", NULL, "UL1_CH4"},
	{"UL1_CH1", "ADDA_UL_CH1", "ADDA Capture"},
	{"UL1_CH2", "ADDA_UL_CH2", "ADDA Capture"},
	{"UL1_CH3", "ADDA_UL_CH1", "ADDA Capture"},
	{"UL1_CH4", "ADDA_UL_CH2", "ADDA Capture"},

	{"UL7", NULL, "UL7_CH1"},
	{"UL7", NULL, "UL7_CH2"},
	{"UL7_CH1", "ADDA_UL_CH1", "ADDA Capture"},
	{"UL7_CH2", "ADDA_UL_CH2", "ADDA Capture"},
};

static const struct snd_soc_component_driver mt6768_afe_pcm_dai_component = {
	.name = "mt6768-afe-pcm-dai",
};

static int mt6768_dai_memif_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mt6768_memif_dai_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mt6768_memif_dai_driver);
	dai->dapm_widgets = mt6768_memif_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mt6768_memif_widgets);
	dai->dapm_routes = mt6768_memif_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mt6768_memif_routes);
	return 0;
}

typedef int (*dai_register_cb)(struct mtk_base_afe *);
static const dai_register_cb dai_register_cbs[] = {
	mt6768_dai_adda_register,
	mt6768_dai_memif_register,
};

static irqreturn_t mt6768_afe_irq_handler(int irq_id, void *dev)
{
	struct mtk_base_afe *afe = dev;
	struct mtk_base_afe_irq *irq = NULL;
	unsigned int status = 0;
	unsigned int status_mcu = 0;
	unsigned int mcu_en = 0;
	int ret = 0;
	int i = 0;

	/* get irq that is sent to MCU */
	regmap_read(afe->regmap, AFE_IRQ_MCU_EN, &mcu_en);

	ret = regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &status);
	/* only care IRQ which is sent to MCU */
	status_mcu = status & mcu_en & AFE_IRQ_STATUS_BITS;

	if (ret || status_mcu == 0) {
		dev_err(afe->dev, "%s(), irq status err, ret %d, status 0x%x, mcu_en 0x%x\n",
			__func__, ret, status, mcu_en);
		goto err_irq;
	}

	for (i = 0; i < MT6768_MEMIF_NUM; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];

		if (!memif->substream)
			continue;

		if (memif->irq_usage < 0)
			continue;

		irq = &afe->irqs[memif->irq_usage];

		if (status_mcu & (1 << irq->irq_data->irq_en_shift))
			snd_pcm_period_elapsed(memif->substream);
	}

err_irq:
	/* clear irq */
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, status_mcu);

	return IRQ_HANDLED;
}

static int mt6768_afe_runtime_suspend(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	unsigned int value = 0;
	int ret;

	/* disable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, 0x1, 0x0);

	ret = regmap_read_poll_timeout(afe->regmap, AFE_DAC_MON, value,
				       (value & AFE_ON_RETM_MASK_SFT) == 0,
				       20, 1 * 1000 * 1000);
	if (ret)
		dev_warn(afe->dev, "%s(), ret %d\n", __func__, ret);

	/* make sure all irq status are cleared */
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, 0xffff);
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, 0xffff);

	mt6768_afe_disable_clock(afe);
	return 0;
}

static int mt6768_afe_runtime_resume(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	int ret;

	ret = mt6768_afe_enable_clock(afe);
	if (ret)
		return ret;

	/* enable audio sys DCM for power saving */
	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0, 0x1 << 29, 0x1 << 29);

	/* force cpu use 8_24 format when writing 32bit data */
	regmap_update_bits(afe->regmap, AFE_MEMIF_MSB, 1 << 28, 0 << 28);

	/* set all output port to 24bit */
	regmap_write(afe->regmap, AFE_CONN_24BIT, 0xffffffff);
	regmap_write(afe->regmap, AFE_CONN_24BIT_1, 0xffffffff);

	/* irq signal to mcu only */
	regmap_write(afe->regmap, AFE_IRQ_MCU_EN, 0xffffffff);

	/* enable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, 0x1, 0x1);

	return 0;
}

static int mt6768_afe_pcm_dev_probe(struct platform_device *pdev)
{
	struct mtk_base_afe *afe;
	struct mt6768_afe_private *afe_priv;
	struct device *dev = &pdev->dev;
	int ret, i, irq_id;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;
	platform_set_drvdata(pdev, afe);

	afe_priv = devm_kzalloc(dev, sizeof(*afe_priv), GFP_KERNEL);
	if (!afe_priv)
		return -ENOMEM;
	afe->platform_priv = afe_priv;
	afe->dev = dev;

	/* init audio related clock */
	ret = mt6768_init_clock(afe);
	if (ret) {
		dev_err(dev, "init clock error\n");
		return ret;
	}

	/* regmap init, afe is a child of the audsys syscon */
	afe->regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(afe->regmap))
		return dev_err_probe(dev, PTR_ERR(afe->regmap),
				     "failed to get regmap\n");

	/* mtkaif pad pinctrl */
	afe_priv->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(afe_priv->pinctrl))
		return dev_err_probe(dev, PTR_ERR(afe_priv->pinctrl),
				     "failed to get pinctrl\n");

	/* init memif */
	afe->memif_32bit_supported = 0;
	afe->memif_size = MT6768_MEMIF_NUM;
	afe->memif = devm_kcalloc(dev, afe->memif_size, sizeof(*afe->memif),
				  GFP_KERNEL);
	if (!afe->memif)
		return -ENOMEM;

	for (i = 0; i < afe->memif_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->memif[i].irq_usage = -1;
	}

	mutex_init(&afe->irq_alloc_lock);

	/* init irq */
	afe->irqs_size = MT6768_IRQ_NUM;
	afe->irqs = devm_kcalloc(dev, afe->irqs_size, sizeof(*afe->irqs),
				 GFP_KERNEL);
	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++)
		afe->irqs[i].irq_data = &irq_data[i];

	irq_id = platform_get_irq(pdev, 0);
	if (irq_id < 0)
		return irq_id;

	ret = devm_request_irq(dev, irq_id, mt6768_afe_irq_handler,
			       IRQF_TRIGGER_NONE, "Afe_ISR_Handle", afe);
	if (ret)
		return dev_err_probe(dev, ret, "could not request irq\n");

	/* init sub_dais */
	INIT_LIST_HEAD(&afe->sub_dais);

	for (i = 0; i < ARRAY_SIZE(dai_register_cbs); i++) {
		ret = dai_register_cbs[i](afe);
		if (ret) {
			dev_warn(dev, "dai register i %d fail, ret %d\n",
				 i, ret);
			return ret;
		}
	}

	/* init dai_driver and component_driver */
	ret = mtk_afe_combine_sub_dai(afe);
	if (ret) {
		dev_warn(dev, "mtk_afe_combine_sub_dai fail, ret %d\n", ret);
		return ret;
	}

	afe->mtk_afe_hardware = &mt6768_afe_hardware;
	afe->memif_fs = mt6768_memif_fs;
	afe->irq_fs = mt6768_irq_fs;
	afe->runtime_suspend = mt6768_afe_runtime_suspend;
	afe->runtime_resume = mt6768_afe_runtime_resume;

	pm_runtime_enable(dev);

	/* register platform */
	ret = devm_snd_soc_register_component(dev, &mtk_afe_pcm_platform,
					      NULL, 0);
	if (ret) {
		dev_warn(dev, "err_platform\n");
		goto err_pm_disable;
	}

	ret = devm_snd_soc_register_component(dev, &mt6768_afe_pcm_dai_component,
					      afe->dai_drivers,
					      afe->num_dai_drivers);
	if (ret) {
		dev_warn(dev, "err_dai_component\n");
		goto err_pm_disable;
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static void mt6768_afe_pcm_dev_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		mt6768_afe_runtime_suspend(&pdev->dev);
}

static const struct of_device_id mt6768_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6768-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6768_afe_pcm_dt_match);

static const struct dev_pm_ops mt6768_afe_pm_ops = {
	SET_RUNTIME_PM_OPS(mt6768_afe_runtime_suspend,
			   mt6768_afe_runtime_resume, NULL)
};

static struct platform_driver mt6768_afe_pcm_driver = {
	.driver = {
		   .name = "mt6768-audio",
		   .of_match_table = mt6768_afe_pcm_dt_match,
		   .pm = &mt6768_afe_pm_ops,
	},
	.probe = mt6768_afe_pcm_dev_probe,
	.remove = mt6768_afe_pcm_dev_remove,
};

module_platform_driver(mt6768_afe_pcm_driver);

MODULE_DESCRIPTION("Mediatek ALSA SoC AFE platform driver for 6768");
MODULE_AUTHOR("Michael Hsiao <michael.hsiao@mediatek.com>");
MODULE_LICENSE("GPL v2");
