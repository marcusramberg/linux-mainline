// SPDX-License-Identifier: GPL-2.0
//
// FourSemi FS1815 boosted amplifier, used here with analog input
//
// Register sequences from the Fourier Semiconductor vendor driver. The
// per-board register tables come from the vendor preset file (fs1815.fsm).
// No DAI: it is an aux device that powers up with whatever feeds IN.

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <sound/soc.h>

#define FS1815_DEVID		0x01
#define FS1815_REVID		0x02
#define FS1815_PWRCTRL		0x10
#define FS1815_SYSCTRL		0x11
#define FS1815_SPKCOEF		0x14
#define FS1815_BSGCTRL		0xbc

#define FS1815_DEVID_VAL	0x12
#define FS1815_FW_NAME		"fs1815.fsm"

#define FSM_DSC_DEV_INFO	0
#define FSM_DSC_SPK_INFO	1
#define FSM_DSC_REG_COMMON	2
#define FSM_DSC_REG_SCENES	3
#define FSM_INFO_SPK_TEMPR_COEF	1
#define FSM_SCENE_MUSIC		BIT(0)

#define FSM_HDR_SIZE		28	/* packed preset_header */
#define FSM_DEV_HDR_SIZE	40	/* packed dev_list, up to index[] */

struct fs1815 {
	struct regmap *regmap;
	struct device *dev;
	unsigned int revid;
	bool inited;
};

/* 16-bit register op: addr[7:0] pos[11:8] (width - 1)[15:12], then value */
static int fs1815_reg_op(struct fs1815 *fs, u16 op, u16 val)
{
	unsigned int addr = op & 0xff, pos = (op >> 8) & 0xf, len = op >> 12;
	unsigned int mask;

	if (pos == 0xf && len == 1) {
		msleep(val);
		return 0;
	}
	if (pos == 0xf)
		return 0;	/* bit waits: none in the lamuc preset */
	if (pos == 0 && len == 15)
		return regmap_write(fs->regmap, addr, val);

	mask = GENMASK(pos + len, pos);
	if (fs->revid == 0x11 && addr == 0x20 && pos <= 8 && pos + len >= 12) {
		/* rev 0x11 remaps BSTCTRL VOUTSEL [12:8] */
		unsigned int v = (val >> (8 - pos)) & 0x1f;

		v = v <= 0x07 ? v : v <= 0x0f ? v + 2 : v <= 0x17 ? v + 4 :
		    v == 0x18 ? 0x1e : 0x1f;
		val = (val & ~(0x1f << (8 - pos))) | (v << (8 - pos));
	} else if (fs->revid == 0x11 && addr >= 0x59 && addr <= 0x5c &&
		   pos == 0 && len >= 3) {
		/* rev 0x11 remaps the AGC [3:0] steps */
		unsigned int v = val & 0xf;

		v = v <= 0x2 ? v * 2 + 1 : v <= 0x4 ? 0x6 : v <= 0x9 ? v + 2 :
		    v <= 0xb ? 0xc : 0xe;
		val = (val & ~0xf) | v;
	}

	return regmap_update_bits(fs->regmap, addr, mask, val << pos);
}

static const u8 *fs1815_find_list(const u8 *dev, size_t room, u16 type)
{
	u16 n = get_unaligned_le16(dev + 26);
	const u8 *index = dev + FSM_DEV_HDR_SIZE;
	int i;

	if (room < FSM_DEV_HDR_SIZE + 4 * n)
		return NULL;
	for (i = 0; i < n; i++) {
		u16 off = get_unaligned_le16(index + 4 * i);

		if (get_unaligned_le16(index + 4 * i + 2) == type &&
		    FSM_DEV_HDR_SIZE + off + 2 <= room)
			return index + off;
	}

	return NULL;
}

static int fs1815_apply_preset(struct fs1815 *fs, const struct firmware *fw)
{
	struct i2c_client *i2c = to_i2c_client(fs->dev);
	const u8 *d = fw->data, *dev = NULL, *list, *end = fw->data + fw->size;
	u16 ndev, n, i;
	int ret;

	if (fw->size < FSM_HDR_SIZE || get_unaligned_le16(d + 22) != fw->size ||
	    crc16(0, d + 26, fw->size - 26) != get_unaligned_le16(d + 24))
		return dev_err_probe(fs->dev, -EINVAL, "bad preset\n");

	ndev = get_unaligned_le16(d + 26);
	for (i = 0; i < ndev && FSM_HDR_SIZE + 4 * (i + 1) <= fw->size; i++) {
		u16 off = get_unaligned_le16(d + FSM_HDR_SIZE + 4 * i);
		const u8 *p = d + off;

		if (get_unaligned_le16(d + FSM_HDR_SIZE + 4 * i + 2) != FSM_DSC_DEV_INFO ||
		    off + FSM_DEV_HDR_SIZE > fw->size)
			continue;
		if (get_unaligned_le16(p + 30) == i2c->addr) {
			dev = p;
			break;
		}
	}
	if (!dev)
		return dev_err_probe(fs->dev, -ENODEV, "no preset for 0x%02x\n", i2c->addr);

	ret = regmap_write(fs->regmap, FS1815_PWRCTRL, 0x0002);
	if (ret)
		return ret;
	msleep(15);
	ret = regmap_write(fs->regmap, FS1815_PWRCTRL, 0x0001);
	if (ret)
		return ret;

	list = fs1815_find_list(dev, end - dev, FSM_DSC_SPK_INFO);
	if (list && get_unaligned_le16(list) > FSM_INFO_SPK_TEMPR_COEF) {
		if (list + 2 + 2 * (FSM_INFO_SPK_TEMPR_COEF + 1) > end)
			return -EINVAL;
		ret = regmap_write(fs->regmap, FS1815_SPKCOEF,
				   get_unaligned_le16(list + 2 + 2 * FSM_INFO_SPK_TEMPR_COEF) << 1);
		if (ret)
			return ret;
	}

	list = fs1815_find_list(dev, end - dev, FSM_DSC_REG_COMMON);
	if (list) {
		n = get_unaligned_le16(list);
		if (list + 2 + 4 * n > end)
			return -EINVAL;
		for (i = 0; i < n && !ret; i++)
			ret = fs1815_reg_op(fs, get_unaligned_le16(list + 2 + 4 * i),
					    get_unaligned_le16(list + 4 + 4 * i));
	}

	list = fs1815_find_list(dev, end - dev, FSM_DSC_REG_SCENES);
	if (list) {
		n = get_unaligned_le16(list);
		if (list + 2 + 6 * n > end)
			return -EINVAL;
		for (i = 0; i < n && !ret; i++) {
			const u8 *r = list + 2 + 6 * i;

			if (get_unaligned_le16(r) & FSM_SCENE_MUSIC)
				ret = fs1815_reg_op(fs, get_unaligned_le16(r + 2),
						    get_unaligned_le16(r + 4));
		}
	}

	return ret;
}

static int fs1815_init(struct fs1815 *fs)
{
	const struct firmware *fw;
	int ret;

	if (fs->inited)
		return 0;

	ret = request_firmware(&fw, FS1815_FW_NAME, fs->dev);
	if (ret)
		return ret;
	ret = fs1815_apply_preset(fs, fw);
	release_firmware(fw);
	if (ret)
		return ret;

	fs->inited = true;
	return 0;
}

static int fs1815_amp_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct fs1815 *fs = snd_soc_component_get_drvdata(component);
	int ret;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = fs1815_init(fs);
		if (ret) {
			dev_err(fs->dev, "init failed: %d\n", ret);
			return ret;
		}
		if (fs->revid == 0x11)
			regmap_write(fs->regmap, FS1815_BSGCTRL, 0x9433);
		regmap_write(fs->regmap, FS1815_SYSCTRL, 0x008f);
		ret = regmap_write(fs->regmap, FS1815_PWRCTRL, 0x0000);
		msleep(10);
		return ret;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_write(fs->regmap, FS1815_PWRCTRL, 0x0001);
		ret = regmap_write(fs->regmap, FS1815_SYSCTRL, 0x0000);
		msleep(10);
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget fs1815_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_OUT_DRV_E("AMP", SND_SOC_NOPM, 0, 0, NULL, 0, fs1815_amp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route fs1815_routes[] = {
	{ "AMP", NULL, "IN" },
	{ "SPK", NULL, "AMP" },
};

static const struct snd_soc_component_driver fs1815_component = {
	.dapm_widgets = fs1815_widgets,
	.num_dapm_widgets = ARRAY_SIZE(fs1815_widgets),
	.dapm_routes = fs1815_routes,
	.num_dapm_routes = ARRAY_SIZE(fs1815_routes),
};

static const struct regmap_config fs1815_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
};

static int fs1815_probe(struct i2c_client *i2c)
{
	struct fs1815 *fs;
	unsigned int id;
	int ret;

	fs = devm_kzalloc(&i2c->dev, sizeof(*fs), GFP_KERNEL);
	if (!fs)
		return -ENOMEM;
	fs->dev = &i2c->dev;
	i2c_set_clientdata(i2c, fs);

	fs->regmap = devm_regmap_init_i2c(i2c, &fs1815_regmap);
	if (IS_ERR(fs->regmap))
		return PTR_ERR(fs->regmap);

	ret = regmap_read(fs->regmap, FS1815_DEVID, &id);
	if (ret)
		return dev_err_probe(fs->dev, ret, "no device\n");
	if ((id & 0xff) != FS1815_DEVID_VAL)
		return dev_err_probe(fs->dev, -ENODEV, "unknown id %04x\n", id);
	regmap_read(fs->regmap, FS1815_REVID, &fs->revid);
	dev_info(fs->dev, "FS1815 id %04x rev %04x\n", id, fs->revid);

	return devm_snd_soc_register_component(fs->dev, &fs1815_component,
					       NULL, 0);
}

static const struct of_device_id fs1815_of_match[] = {
	{ .compatible = "foursemi,fs1815" },
	{ }
};
MODULE_DEVICE_TABLE(of, fs1815_of_match);

static struct i2c_driver fs1815_driver = {
	.driver = {
		.name = "fs1815",
		.of_match_table = fs1815_of_match,
	},
	.probe = fs1815_probe,
};
module_i2c_driver(fs1815_driver);

MODULE_DESCRIPTION("FourSemi FS1815 amplifier driver");
MODULE_FIRMWARE(FS1815_FW_NAME);
MODULE_LICENSE("GPL");
