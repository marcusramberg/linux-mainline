// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MT6768 CONNSYS Wi-Fi: an MT7663 core on the SoC bus, in the CONNSYS
 * window, behind an MCU it shares with Bluetooth.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pwrseq/consumer.h>
#include "mt7615.h"

static void mt6768_wmac_power_off(void *pwrseq)
{
	pwrseq_power_off(pwrseq);
}

static int mt6768_wmac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_desc *pwrseq;
	void __iomem *mem_base;
	struct resource *res;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* the parent CONNSYS node owns the window */
	res = platform_get_resource(to_platform_device(dev->parent),
				    IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	mem_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!mem_base)
		return -ENOMEM;

	pwrseq = devm_pwrseq_get(dev, "wlan");
	if (IS_ERR(pwrseq))
		return dev_err_probe(dev, PTR_ERR(pwrseq), "no CONNSYS\n");

	ret = pwrseq_power_on(pwrseq);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mt6768_wmac_power_off, pwrseq);
	if (ret)
		return ret;

	return mt7615_mmio_probe(dev, mem_base, irq, mt6768_reg_map, 0x6768);
}

static void mt6768_wmac_remove(struct platform_device *pdev)
{
	struct mt7615_dev *dev = platform_get_drvdata(pdev);

	mt7615_unregister_device(dev);
}

static const struct of_device_id mt6768_wmac_of_match[] = {
	{ .compatible = "mediatek,mt6768-wmac" },
	{},
};

struct platform_driver mt6768_wmac_driver = {
	.driver = {
		.name = "mt6768-wmac",
		.of_match_table = mt6768_wmac_of_match,
	},
	.probe = mt6768_wmac_probe,
	.remove = mt6768_wmac_remove,
};

MODULE_FIRMWARE(MT6768_FIRMWARE_N9);
MODULE_FIRMWARE(MT6768_NVRAM);
