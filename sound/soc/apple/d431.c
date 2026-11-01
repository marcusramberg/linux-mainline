// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple D431 (iPhone 11 Pro Max) sound card: the top speaker amp on MCA
 * cluster 5, as a DPCM front end (mca-pcm-5) and back end (mca-i2s-5). The
 * cs42l77 is the bus clock master; the AOP's MCA pin pages carry its MCLK and
 * the AP's TX data to the board, and the AOP firmware is not needed for that.
 */

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

SND_SOC_DAILINK_DEFS(fe,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(be,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-i2s-5")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "cs35l27-asp")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

#define AOP_PS_STRIDE		0x8
#define  AOP_PS_DESIRED		GENMASK(3, 0)
#define  AOP_PS_ACTUAL		GENMASK(7, 4)
#define  AOP_PS_KEEP		0xfffffcf0
#define AOP_MCA_STRIDE		0x4000
#define AOP_PIN_ENABLES		0x600
#define  AOP_PIN_MCLK		BIT(0)
#define  AOP_PIN_DOUT		BIT(3)
#define AOP_PIN_CLK_SEL		0x604
#define  AOP_PIN_CLK_SEL_IOS	0x700
#define AOP_PIN_DATA_SEL	0x608
#define  AOP_PIN_DATA_AP	0x8000 /* the AP's MCA, as iOS */

static void __iomem *d431_map(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* The sound node is outside /soc: posted writes raise an SError */
	res->flags |= IORESOURCE_MEM_NONPOSTED;
	return devm_ioremap_resource(&pdev->dev, res);
}

/* AOP MCA0 drives the codec's MCLK, AOP MCA1 the amp's SDIN */
static int d431_aop_pins_start(struct platform_device *pdev)
{
	static const u32 enables[] = {
		AOP_PIN_MCLK | AOP_PIN_DOUT, AOP_PIN_DOUT,
	};
	void __iomem *ps = d431_map(pdev, "aop-ps");
	void __iomem *mca = d431_map(pdev, "aop-mca");
	int i, ret;
	u32 val;

	if (IS_ERR(ps))
		return PTR_ERR(ps);
	if (IS_ERR(mca))
		return PTR_ERR(mca);

	for (i = 0; i < ARRAY_SIZE(enables); i++) {
		void __iomem *reg = ps + i * AOP_PS_STRIDE;
		void __iomem *pin = mca + i * AOP_MCA_STRIDE;

		writel((readl(reg) & AOP_PS_KEEP) | AOP_PS_DESIRED, reg);
		ret = readl_poll_timeout(reg, val,
					 (val & AOP_PS_ACTUAL) == AOP_PS_ACTUAL,
					 10, 10000);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "AOP MCA%d power\n", i);

		writel(AOP_PIN_CLK_SEL_IOS, pin + AOP_PIN_CLK_SEL);
		writel(AOP_PIN_DATA_AP, pin + AOP_PIN_DATA_SEL);
		writel(enables[i], pin + AOP_PIN_ENABLES);
	}
	return 0;
}

static int d431_fe_init(struct snd_soc_pcm_runtime *rtd)
{
	/* As iOS: 15 16-bit slots in the 250-cycle frame; the amp reads slot 2 and sends V, I in 11, 12 */
	return snd_soc_dai_set_tdm_slot(snd_soc_rtd_to_cpu(rtd, 0), 0x4, 0x1800, 15, 16);
}

static struct snd_soc_dai_link d431_links[] = {
	{
		.name = "Speaker",
		.stream_name = "Speaker",
		.dynamic = 1,
		.init = d431_fe_init,
		SND_SOC_DAILINK_REG(fe),
	},
	{
		.name = "Speaker Amp",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		SND_SOC_DAILINK_REG(be),
	},
};

static struct snd_soc_aux_dev d431_aux = {};

static const struct snd_soc_dapm_route d431_routes[] = {
	{ "I2S5 TX", NULL, "PCM5 TX" },
	{ "PCM5 RX", NULL, "I2S5 RX" },
};

static struct snd_soc_card d431_card = {
	.name = "D431",
	.owner = THIS_MODULE,
	.dai_link = d431_links,
	.num_links = ARRAY_SIZE(d431_links),
	.aux_dev = &d431_aux,
	.num_aux_devs = 1,
	.dapm_routes = d431_routes,
	.num_dapm_routes = ARRAY_SIZE(d431_routes),
};

static int d431_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *mca, *amp, *codec, *dma;
	struct platform_device *dma_pdev;
	bool ready;
	int ret;

	mca = of_parse_phandle(np, "apple,mca", 0);
	amp = of_parse_phandle(np, "apple,speaker-amp", 0);
	codec = of_parse_phandle(np, "apple,codec", 0);
	if (!mca || !amp || !codec)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing mca, amp or codec\n");

	/* ASoC does not retry a card when a DMA controller shows up */
	dma = of_parse_phandle(mca, "dmas", 0);
	dma_pdev = dma ? of_find_device_by_node(dma) : NULL;
	of_node_put(dma);
	if (!dma_pdev)
		return -EPROBE_DEFER;
	ready = device_is_bound(&dma_pdev->dev);
	put_device(&dma_pdev->dev);
	if (!ready)
		return -EPROBE_DEFER;

	d431_links[0].cpus->of_node = mca;
	d431_links[0].platforms->of_node = mca;
	d431_links[1].cpus->of_node = mca;
	d431_links[1].platforms->of_node = mca;
	d431_links[1].codecs->of_node = amp;

	d431_aux.dlc.of_node = codec;

	ret = d431_aop_pins_start(pdev);
	if (ret)
		return ret;

	d431_card.dev = &pdev->dev;
	return devm_snd_soc_register_card(&pdev->dev, &d431_card);
}

static const struct of_device_id d431_of_match[] = {
	{ .compatible = "apple,d431-sound" },
	{}
};
MODULE_DEVICE_TABLE(of, d431_of_match);

static struct platform_driver d431_driver = {
	.driver = {
		.name = "snd-soc-d431",
		.of_match_table = d431_of_match,
	},
	.probe = d431_probe,
};
module_platform_driver(d431_driver);

MODULE_DESCRIPTION("Apple D431 sound card");
MODULE_LICENSE("GPL");
