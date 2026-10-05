// SPDX-License-Identifier: GPL-2.0
//
// mt6768-mt6358.c  --  mt6768 mt6358 ALSA SoC machine driver
//
// Copyright (c) 2022 MediaTek Inc.
// Author: Gang Feng <gang.feng@mediatek.com>

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "mt6768-afe-common.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../../codecs/mt6358.h"
#include "../../codecs/mt6358-accdet.h"

static struct snd_soc_component *mt6768_accdet;
static struct snd_soc_jack mt6768_headset;

static struct snd_soc_jack_pin mt6768_headset_pins[] = {
	{ .pin = "Headphone", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Headset Mic", .mask = SND_JACK_MICROPHONE },
};

static const struct snd_soc_dapm_widget mt6768_mt6358_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_soc_dapm_route mt6768_mt6358_routes[] = {
	{ "Headphone", NULL, "Headphone L" },
	{ "Headphone", NULL, "Headphone R" },
	{ "AIN1", NULL, "Headset Mic" },
};

static int mt6768_mt6358_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec_component =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	int ret;

	/* set mtkaif protocol */
	mt6358_set_mtkaif_protocol(codec_component,
				   MT6358_MTKAIF_PROTOCOL_1);

	if (!mt6768_accdet)
		return 0;

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2,
					 &mt6768_headset, mt6768_headset_pins,
					 ARRAY_SIZE(mt6768_headset_pins));
	if (ret)
		return ret;

	return mt6358_accdet_enable_jack_detect(mt6768_accdet, &mt6768_headset);
}

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
SND_SOC_DAILINK_DEFS(capture1,
	DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
SND_SOC_DAILINK_DEFS(adda,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mt6358-sound",
				      "mt6358-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mt6768_mt6358_dai_links[] = {
	/* Front End DAI links */
	{
		.name = "Playback_1",
		.stream_name = "Playback_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_merged_rate = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback1),
	},
	{
		.name = "Capture_1",
		.stream_name = "Capture_1",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.dpcm_merged_rate = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture1),
	},
	/* Back End DAI links */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.init = mt6768_mt6358_init,
		SND_SOC_DAILINK_REG(adda),
	},
};

/*
 * The speaker amps have no feedback loop to the host, only their own
 * AGC/thermal limits: cap the analog gain at the stock maximum
 * (-1 dB line out, 0 dB receiver).
 */
static int mt6768_mt6358_late_probe(struct snd_soc_card *card)
{
	int ret;

	/* > 0 just means the current value got clipped */
	ret = snd_soc_limit_volume(card, "Lineout Volume", 9);
	if (ret >= 0)
		ret = snd_soc_limit_volume(card, "Handset Volume", 10);

	return min(ret, 0);
}

/* the jack goes away with the card, ACCDET does not */
static int mt6768_mt6358_remove(struct snd_soc_card *card)
{
	if (mt6768_accdet)
		mt6358_accdet_enable_jack_detect(mt6768_accdet, NULL);

	return 0;
}

static struct snd_soc_card mt6768_mt6358_soc_card = {
	.name = "mt6768-mt6358",
	.owner = THIS_MODULE,
	.late_probe = mt6768_mt6358_late_probe,
	.remove = mt6768_mt6358_remove,
	.dapm_widgets = mt6768_mt6358_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6768_mt6358_widgets),
	.dapm_routes = mt6768_mt6358_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6768_mt6358_routes),
	.dai_link = mt6768_mt6358_dai_links,
	.num_links = ARRAY_SIZE(mt6768_mt6358_dai_links),
};

static int mt6768_mt6358_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6768_mt6358_soc_card;
	struct device_node *platform_node, *np;
	struct platform_device *accdet_pdev;
	struct snd_soc_dai_link *dai_link;
	int ret, i;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'mediatek,platform' missing or invalid\n");
		return -EINVAL;
	}

	for_each_card_prelinks(card, i, dai_link) {
		if (!dai_link->platforms->name)
			dai_link->platforms->of_node = platform_node;
	}

	card->dev = &pdev->dev;

	np = of_parse_phandle(pdev->dev.of_node, "mediatek,accdet", 0);
	if (np) {
		accdet_pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (accdet_pdev) {
			mt6768_accdet = snd_soc_lookup_component(&accdet_pdev->dev, NULL);
			put_device(&accdet_pdev->dev);
		}
		if (!mt6768_accdet) {
			of_node_put(platform_node);
			return -EPROBE_DEFER;
		}
	}

	/* analog speaker amps behind the codec's line/receiver outputs */
	ret = snd_soc_of_parse_aux_devs(card, "audio-aux-devs");
	if (!ret)
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret) {
		of_node_put(platform_node);
		return dev_err_probe(&pdev->dev, ret, "aux devs/routing\n");
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	of_node_put(platform_node);
	if (ret)
		dev_err_probe(&pdev->dev, ret, "snd_soc_register_card fail\n");

	return ret;
}

static const struct of_device_id mt6768_mt6358_dt_match[] = {
	{ .compatible = "mediatek,mt6768-mt6358-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6768_mt6358_dt_match);

static struct platform_driver mt6768_mt6358_driver = {
	.driver = {
		.name = "mt6768-mt6358",
		.of_match_table = mt6768_mt6358_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = mt6768_mt6358_dev_probe,
};

module_platform_driver(mt6768_mt6358_driver);

/* Module information */
MODULE_DESCRIPTION("MT6768 MT6358 ALSA SoC machine driver");
MODULE_AUTHOR("Gang Feng <gang.feng@mediatek.com>");
MODULE_LICENSE("GPL v2");
