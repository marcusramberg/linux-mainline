// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6768 CONNSYS (Wi-Fi/BT/GPS/FM subsystem) power-on, after the
 * vendor's wmt_drv platform/mt6768.c.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#define CONSYS_HW_ID		0x000	/* mcu */
#define CONSYS_FW_ID		0x004	/* mcu */
#define CONSYS_IP_VER		0x010	/* misc */
#define CONSYS_IP_VER_MT6768	0x10020501

struct mtk_consys {
	struct device *dev;
	void __iomem *mcu;
	void __iomem *misc;
	struct regulator *vcn18;
	struct reset_control *cpu_rst;
	struct reset_control *sw_rst;
};

static int mtk_consys_power_on(struct mtk_consys *cs)
{
	u32 ver;
	int ret;

	ret = regulator_enable(cs->vcn18);
	if (ret)
		return ret;
	udelay(150);

	reset_control_assert(cs->cpu_rst);

	ret = pm_runtime_resume_and_get(cs->dev);
	if (ret)
		goto err_vcn18;

	reset_control_deassert(cs->sw_rst);
	udelay(10);

	ret = readl_poll_timeout(cs->misc + CONSYS_IP_VER, ver,
				 ver == CONSYS_IP_VER_MT6768, 1000, 200000);
	dev_info(cs->dev, "IP version %08x, HW %04x, FW %04x\n", ver,
		 readl(cs->mcu + CONSYS_HW_ID) & 0xffff,
		 readl(cs->mcu + CONSYS_FW_ID) & 0xffff);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	reset_control_assert(cs->sw_rst);
	pm_runtime_put_sync(cs->dev);
err_vcn18:
	regulator_disable(cs->vcn18);
	return ret;
}

static int mtk_consys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_consys *cs;
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->dev = dev;

	cs->mcu = devm_platform_ioremap_resource_byname(pdev, "mcu");
	if (IS_ERR(cs->mcu))
		return PTR_ERR(cs->mcu);
	cs->misc = devm_platform_ioremap_resource_byname(pdev, "misc");
	if (IS_ERR(cs->misc))
		return PTR_ERR(cs->misc);

	cs->vcn18 = devm_regulator_get(dev, "vcn18");
	if (IS_ERR(cs->vcn18))
		return PTR_ERR(cs->vcn18);
	cs->cpu_rst = devm_reset_control_get_exclusive(dev, "cpu");
	if (IS_ERR(cs->cpu_rst))
		return PTR_ERR(cs->cpu_rst);
	cs->sw_rst = devm_reset_control_get_exclusive(dev, "sw");
	if (IS_ERR(cs->sw_rst))
		return PTR_ERR(cs->sw_rst);

	platform_set_drvdata(pdev, cs);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	return mtk_consys_power_on(cs);
}

static const struct of_device_id mtk_consys_of_match[] = {
	{ .compatible = "mediatek,mt6768-consys" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_consys_of_match);

static struct platform_driver mtk_consys_driver = {
	.probe = mtk_consys_probe,
	.driver = {
		.name = "mtk-consys",
		.of_match_table = mtk_consys_of_match,
	},
};
module_platform_driver(mtk_consys_driver);

MODULE_DESCRIPTION("MediaTek CONNSYS power-on");
MODULE_LICENSE("GPL");
