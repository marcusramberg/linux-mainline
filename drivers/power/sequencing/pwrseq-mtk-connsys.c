// SPDX-License-Identifier: GPL-2.0
/*
 * Power sequencer for the CONNSYS block of MediaTek MT6768 (CONNAC1)
 *
 * CONNSYS has one MCU for Bluetooth, Wi-Fi, GPS and FM. Powering it up
 * means booting that MCU from ROM images the host places in reserved
 * memory; its consumers then patch and configure it over BTIF.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwrseq/provider.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#define CONSYS_EMI_CTL			0x02150
#define CONSYS_EMI_CTL_SW		BIT(21)
#define CONSYS_ROM_STATE		0x02600
#define CONSYS_ROM_DONE			0x1d1e
#define CONSYS_IP_VER			0xb1010
#define CONSYS_IP_VER_MT6768		0x10020501
#define CONSYS_AFE_WPLL			0xb3034
#define CONSYS_CO_CLK0			0xc1200
#define CONSYS_CO_CLK1			0xc1204
#define CONSYS_RESTORE			0xc1600
#define CONSYS_RESTORE_DONE		BIT(23)

#define CONSYS_LOG			0x36500
#define CONSYS_LOG_SIZE			SZ_192K
#define CONSYS_LOG_READY		0x38

/* the MCU sees EMI and pericfg through windows the AP points at them */
#define INFRA_CONN_EMI_REMAP		0x380
#define INFRA_CONN_EMI_REMAP_BASE	GENMASK(11, 0)
#define INFRA_CONN_PERI_REMAP		0x388
#define INFRA_CONN_PERI_REMAP_BASE	GENMASK(27, 16)
#define PERICFG_BASE			0x10003000

#define FW_ROM_MCU	"mediatek/soc1_0_ram_mcu_1a_1_hdr.bin"
#define FW_ROM_BT	"mediatek/soc1_0_ram_bt_1a_1_hdr.bin"
#define FW_ROM_WIFI	"mediatek/soc1_0_ram_wifi_1a_1_hdr.bin"

struct consys_rom_hdr {
	u8 date[16];
	u8 platform[4];
	__le16 hw_ver;
	__le16 sw_ver;
	__le32 addr;	/* 0xf0xxxx11, the low byte is a flag */
	__le32 type;
	__le32 crc[4];
} __packed;

struct pwrseq_mtk_connsys {
	struct device *dev;
	void __iomem *base;
	struct regmap *infracfg;
	struct resource emi;
	struct regulator *vcn18;
	struct regulator *vcn28;
	struct regulator *vcn33;
	struct reset_control *cpu_rst;
	struct reset_control *sw_rst;
};

static void consys_rmw(struct pwrseq_mtk_connsys *cs, u32 reg, u32 mask,
		       u32 val)
{
	writel((readl(cs->base + reg) & ~mask) | val, cs->base + reg);
}

static int consys_load_rom(struct pwrseq_mtk_connsys *cs, const char *name)
{
	const struct consys_rom_hdr *hdr;
	const struct firmware *fw;
	size_t offs, len;
	void *emi;
	int ret;

	ret = request_firmware(&fw, name, cs->dev);
	if (ret)
		return ret;

	ret = -EINVAL;
	if (fw->size < sizeof(*hdr))
		goto out;
	hdr = (const void *)fw->data;
	offs = le32_to_cpu(hdr->addr) & 0xffff00;
	len = fw->size - sizeof(*hdr);
	if (offs + len > resource_size(&cs->emi))
		goto out;

	ret = -ENOMEM;
	emi = memremap(cs->emi.start + offs, len, MEMREMAP_WC);
	if (!emi)
		goto out;
	memcpy(emi, fw->data + sizeof(*hdr), len);
	memunmap(emi);
	ret = 0;
out:
	if (ret)
		dev_err(cs->dev, "%s: cannot load (%d)\n", name, ret);
	release_firmware(fw);
	return ret;
}

/*
 * The MCU logs into rings in its EMI, at the offsets the host leaves in this
 * block; with none set up it writes over its own code at offset 0.
 */
static int consys_log_init(struct pwrseq_mtk_connsys *cs)
{
	static const u32 ring[][2] = {	/* MCU, Wi-Fi, BT, GPS */
		{ 0x40, 0x4000 }, { 0x4080, 0x10000 }, { 0x140c0, 0x10000 },
		{ 0x24100, 0x8000 },
	};
	void *log;
	int i;

	log = memremap(cs->emi.start + CONSYS_LOG, CONSYS_LOG_SIZE,
		       MEMREMAP_WC);
	if (!log)
		return -ENOMEM;

	memset(log, 0, CONSYS_LOG_SIZE);
	for (i = 0; i < ARRAY_SIZE(ring); i++) {
		put_unaligned_le32(ring[i][0], log + 8 * i);
		put_unaligned_le32(ring[i][1], log + 8 * i + 4);
	}
	memcpy(log + CONSYS_LOG_READY, "EMIFWLOG", 8);
	memunmap(log);

	return 0;
}

static int consys_remap(struct pwrseq_mtk_connsys *cs)
{
	int ret;

	ret = regmap_update_bits(cs->infracfg, INFRA_CONN_EMI_REMAP,
				 INFRA_CONN_EMI_REMAP_BASE,
				 FIELD_PREP(INFRA_CONN_EMI_REMAP_BASE,
					    cs->emi.start >> 24));
	if (ret)
		return ret;

	return regmap_update_bits(cs->infracfg, INFRA_CONN_PERI_REMAP,
				  INFRA_CONN_PERI_REMAP_BASE,
				  FIELD_PREP(INFRA_CONN_PERI_REMAP_BASE,
					     PERICFG_BASE >> 24));
}

static int consys_boot(struct pwrseq_mtk_connsys *cs)
{
	static const char * const roms[] = {
		FW_ROM_MCU, FW_ROM_BT, FW_ROM_WIFI,
	};
	u32 val;
	int i, ret;

	ret = readl_poll_timeout(cs->base + CONSYS_IP_VER, val,
				 val == CONSYS_IP_VER_MT6768, 1000, 200000);
	if (ret) {
		dev_err(cs->dev, "unexpected IP version %08x\n", val);
		return ret;
	}

	ret = consys_remap(cs);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(roms); i++) {
		ret = consys_load_rom(cs, roms[i]);
		if (ret)
			return ret;
	}

	ret = consys_log_init(cs);
	if (ret)
		return ret;

	consys_rmw(cs, CONSYS_CO_CLK0, 0xffff, 0x708);
	consys_rmw(cs, CONSYS_CO_CLK1, 0xff01, 0x600);
	consys_rmw(cs, CONSYS_EMI_CTL, CONSYS_EMI_CTL_SW, 0);
	consys_rmw(cs, CONSYS_RESTORE, 0, CONSYS_RESTORE_DONE);
	consys_rmw(cs, CONSYS_AFE_WPLL, GENMASK(20, 19), BIT(19));

	ret = reset_control_deassert(cs->cpu_rst);
	if (ret)
		return ret;

	ret = readl_poll_timeout(cs->base + CONSYS_ROM_STATE, val,
				 val == CONSYS_ROM_DONE, 1000, 200000);
	if (ret) {
		/* 0x4fd0: still in ROM, the EMI images are not where it looks */
		dev_err(cs->dev, "MCU did not boot, ROM state %04x\n", val);
		return ret;
	}
	msleep(20);

	return 0;
}

static void consys_power_down(struct pwrseq_mtk_connsys *cs)
{
	reset_control_assert(cs->cpu_rst);
	/* with the SW reset asserted, CONN cannot ack the bus protection */
	pm_runtime_put_sync(cs->dev);
	regulator_disable(cs->vcn28);
	regulator_disable(cs->vcn33);
	regulator_disable(cs->vcn18);
}

static int pwrseq_mtk_connsys_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_mtk_connsys *cs = pwrseq_device_get_drvdata(pwrseq);
	int ret;

	ret = regulator_enable(cs->vcn18);
	if (ret)
		return ret;
	ret = regulator_enable(cs->vcn33);
	if (ret)
		goto err_vcn18;
	fsleep(150);
	ret = regulator_enable(cs->vcn28);
	if (ret)
		goto err_vcn33;

	ret = reset_control_assert(cs->cpu_rst);
	if (ret)
		goto err_vcn28;
	ret = pm_runtime_resume_and_get(cs->dev);
	if (ret)
		goto err_vcn28;
	ret = reset_control_deassert(cs->sw_rst);
	if (ret)
		goto err_down;
	fsleep(10);

	ret = consys_boot(cs);
	if (ret)
		goto err_down;

	return 0;

err_down:
	consys_power_down(cs);
	return ret;
err_vcn28:
	regulator_disable(cs->vcn28);
err_vcn33:
	regulator_disable(cs->vcn33);
err_vcn18:
	regulator_disable(cs->vcn18);
	return ret;
}

static int pwrseq_mtk_connsys_disable(struct pwrseq_device *pwrseq)
{
	consys_power_down(pwrseq_device_get_drvdata(pwrseq));
	return 0;
}

static const struct pwrseq_unit_data pwrseq_mtk_connsys_unit = {
	.name = "mcu",
	.enable = pwrseq_mtk_connsys_enable,
	.disable = pwrseq_mtk_connsys_disable,
};

static const struct pwrseq_target_data pwrseq_mtk_connsys_bt = {
	.name = "bluetooth",
	.unit = &pwrseq_mtk_connsys_unit,
};

static const struct pwrseq_target_data pwrseq_mtk_connsys_wlan = {
	.name = "wlan",
	.unit = &pwrseq_mtk_connsys_unit,
};

static const struct pwrseq_target_data *pwrseq_mtk_connsys_targets[] = {
	&pwrseq_mtk_connsys_bt,
	&pwrseq_mtk_connsys_wlan,
	NULL
};

static int pwrseq_mtk_connsys_match(struct pwrseq_device *pwrseq,
				    struct device *dev)
{
	struct pwrseq_mtk_connsys *cs = pwrseq_device_get_drvdata(pwrseq);
	struct device_node *np;
	int ret;

	if (dev->parent == cs->dev)
		return PWRSEQ_MATCH_OK;

	np = of_parse_phandle(dev->of_node, "mediatek,connsys", 0);
	ret = np == cs->dev->of_node ? PWRSEQ_MATCH_OK : PWRSEQ_NO_MATCH;
	of_node_put(np);

	return ret;
}

static int pwrseq_mtk_connsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_config config = {};
	struct pwrseq_mtk_connsys *cs;
	struct pwrseq_device *pwrseq;
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->dev = dev;

	cs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cs->base))
		return PTR_ERR(cs->base);

	cs->infracfg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "mediatek,infracfg");
	if (IS_ERR(cs->infracfg))
		return dev_err_probe(dev, PTR_ERR(cs->infracfg),
				     "no infracfg\n");

	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &cs->emi);
	if (ret || !IS_ALIGNED(cs->emi.start, SZ_16M))
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "needs a 16 MiB aligned memory-region\n");

	cs->vcn18 = devm_regulator_get(dev, "vcn18");
	if (IS_ERR(cs->vcn18))
		return dev_err_probe(dev, PTR_ERR(cs->vcn18), "no vcn18\n");
	cs->vcn28 = devm_regulator_get(dev, "vcn28");
	if (IS_ERR(cs->vcn28))
		return dev_err_probe(dev, PTR_ERR(cs->vcn28), "no vcn28\n");
	cs->vcn33 = devm_regulator_get(dev, "vcn33");
	if (IS_ERR(cs->vcn33))
		return dev_err_probe(dev, PTR_ERR(cs->vcn33), "no vcn33\n");

	cs->cpu_rst = devm_reset_control_get_exclusive(dev, "cpu");
	if (IS_ERR(cs->cpu_rst))
		return dev_err_probe(dev, PTR_ERR(cs->cpu_rst),
				     "no cpu reset\n");
	cs->sw_rst = devm_reset_control_get_exclusive(dev, "sw");
	if (IS_ERR(cs->sw_rst))
		return dev_err_probe(dev, PTR_ERR(cs->sw_rst),
				     "no sw reset\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = cs;
	config.match = pwrseq_mtk_connsys_match;
	config.targets = pwrseq_mtk_connsys_targets;

	pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(pwrseq))
		return dev_err_probe(dev, PTR_ERR(pwrseq),
				     "cannot register power sequencer\n");

	return devm_of_platform_populate(dev);
}

static const struct of_device_id pwrseq_mtk_connsys_of_match[] = {
	{ .compatible = "mediatek,mt6768-connsys" },
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_mtk_connsys_of_match);

static struct platform_driver pwrseq_mtk_connsys_driver = {
	.probe = pwrseq_mtk_connsys_probe,
	.driver = {
		.name = "pwrseq-mtk-connsys",
		.of_match_table = pwrseq_mtk_connsys_of_match,
	},
};
module_platform_driver(pwrseq_mtk_connsys_driver);

MODULE_DESCRIPTION("MediaTek CONNSYS power sequencer");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(FW_ROM_MCU);
MODULE_FIRMWARE(FW_ROM_BT);
MODULE_FIRMWARE(FW_ROM_WIFI);
