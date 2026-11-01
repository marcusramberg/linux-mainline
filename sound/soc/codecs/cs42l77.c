// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cirrus Logic CS42L77 (Apple D431): only what the speaker bus needs, the
 * codec's ASP2 as BCLK/FSYNC master, in the order of iOS AppleCS42L77Audio
 * and the AOP firmware. The core clock is MCLK from the AOP MCA, so this
 * runs as a card aux device, after the card has started it.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <sound/soc.h>

#define CS42L77_DEVID		0x00000
#define  DEVID_CS42L77		0x42a77
#define CS42L77_CORE_CLK	0x00008
#define  CORE_CLK_CCM		BIT(1)
#define CS42L77_PDN1		0x10000
#define CS42L77_PDN2		0x1000c
#define CS42L77_CLK_SRC		0x20000
#define  CLK_SRC_MASK		0x31 /* 0: MCLK1 */
#define CS42L77_ASP2_RATE	0x2000c
#define CS42L77_ASP2_MCLK_DIV	0x20010
#define CS42L77_OTP_STATUS	0x40048
#define  OTP_DONE		BIT(4)
#define CS42L77_INT_40148	0x40148
#define CS42L77_INT_400B4	0x400b4
#define CS42L77_PAD_DRIVE	0x50004
#define CS42L77_DEFAULTS	0x5000c
#define CS42L77_OTP_INIT	0x9004c
#define CS42L77_ASP2_ENABLE	0x4b0000
#define  ASP2_FSYNC_BCLK_EN	GENMASK(1, 0)
#define CS42L77_ASP2_PERIOD	0x4b0004
#define CS42L77_ASP2_FSYNC	0x4b0008
#define CS42L77_ASP2_CFG	0x4b0018

/* control bytes, outside the register space */
#define CTRL_STATUS		0x90
#define  STATUS_READY		0x90
#define CTRL_SET		0xa1
#define  CTRL_PLL_KEEP		0x70 /* 0x60 does not lock */
#define  CTRL_RUN		0x50
#define  CTRL_RUN_DONE		0x40

struct cs42l77 {
	struct spi_device *spi;
	struct gpio_desc *reset;
	u8 *tx, *rx;
};

static int cs42l77_xfer(struct cs42l77 *cs, int len, bool keep_cs)
{
	struct spi_transfer t = {
		.tx_buf = cs->tx, .rx_buf = cs->rx,
		.len = len, .cs_change = keep_cs,
	};

	return spi_sync_transfer(cs->spi, &t, 1);
}

static void cs42l77_cs_off(struct cs42l77 *cs)
{
	struct spi_transfer t = {};

	spi_sync_transfer(cs->spi, &t, 1);
}

static int cs42l77_ctrl_set(struct cs42l77 *cs, u8 val)
{
	cs->tx[0] = CTRL_SET;
	cs->tx[1] = val;
	return cs42l77_xfer(cs, 2, false);
}

static void cs42l77_poll_fill(u8 *p, int len)
{
	int i;

	for (i = 0; i < len; i += 2) {
		p[i] = CTRL_STATUS;
		p[i + 1] = 0;
	}
}

static bool cs42l77_ready(const u8 *rx)
{
	int i;

	for (i = 0; i < 10; i++)
		if ((rx[2 * i] & STATUS_READY) == STATUS_READY)
			return true;
	return false;
}

static int cs42l77_wait(struct cs42l77 *cs)
{
	int i, ret;

	for (i = 0; i < 5; i++) {
		cs42l77_poll_fill(cs->tx, 20);
		ret = cs42l77_xfer(cs, 20, true);
		if (ret)
			return ret;
		if (cs42l77_ready(cs->rx))
			return 0;
	}
	return -ETIMEDOUT;
}

static void cs42l77_addr(u8 *t, u32 reg)
{
	t[0] = 0x80;
	t[1] = reg;
	t[2] = reg >> 8;
	t[3] = reg >> 16;
	t[4] = 0x81;
	t[5] = 1;
	t[6] = 0;
}

static int cs42l77_write(struct cs42l77 *cs, u32 reg, u32 val)
{
	u8 *t = cs->tx;
	int ret;

	cs42l77_addr(t, reg);
	cs42l77_poll_fill(t + 7, 20);
	ret = cs42l77_xfer(cs, 27, true);
	if (!ret && !cs42l77_ready(cs->rx + 8))
		ret = cs42l77_wait(cs);
	if (ret)
		goto out;

	t[0] = 0xb2;
	put_unaligned_le32(val, t + 1);
	cs42l77_poll_fill(t + 5, 20);
	ret = cs42l77_xfer(cs, 25, true);
	if (!ret && !cs42l77_ready(cs->rx + 6))
		ret = cs42l77_wait(cs);
out:
	cs42l77_cs_off(cs);
	return ret;
}

static int cs42l77_read(struct cs42l77 *cs, u32 reg, u32 *val)
{
	u8 *t = cs->tx;
	int i, ret;

	memset(t, 0, 19);
	cs42l77_addr(t, reg);
	t[7] = 0xc4;
	t[17] = 0xc4;
	ret = cs42l77_xfer(cs, 19, true);
	for (i = 0; !ret && (cs->rx[i ? 11 : 18] & 0x50) != 0x50; i++) {
		if (i == 31) {
			ret = -ETIMEDOUT;
			break;
		}
		memset(t, 0, 12);
		t[10] = 0xc4;
		ret = cs42l77_xfer(cs, 12, true);
	}
	if (!ret) {
		memset(t, 0, 6);
		ret = cs42l77_xfer(cs, 6, true);
		*val = get_unaligned_le32(cs->rx);
	}
	cs42l77_cs_off(cs);
	return ret;
}

static int cs42l77_update(struct cs42l77 *cs, u32 reg, u32 mask, u32 val)
{
	u32 old;
	int ret;

	ret = cs42l77_read(cs, reg, &old);
	return ret ?: cs42l77_write(cs, reg, (old & ~mask) | val);
}

static int cs42l77_start(struct cs42l77 *cs)
{
	struct device *dev = &cs->spi->dev;
	static const struct { u32 reg, mask, val; } seq[] = {
		{ CS42L77_DEFAULTS, 0x0cfff038, 0x086db010 },
		{ CS42L77_PAD_DRIVE, 0x38fc0000, 0x186c0000 },
		{ CS42L77_PDN1, BIT(0), BIT(0) },
		{ CS42L77_PDN2, BIT(0), BIT(0) },
		{ CS42L77_INT_40148, BIT(0), 0 },
		{ CS42L77_INT_400B4, BIT(9), 0 },
		{ CS42L77_ASP2_CFG, 0x3f, 0x3b },
		{ CS42L77_ASP2_RATE, 0x3fff3fff, 0x00010001 },
		{ CS42L77_ASP2_MCLK_DIV, 0x7, 0x6 },
		{ CS42L77_ASP2_PERIOD, 0x7ffff, 250 },
		{ CS42L77_ASP2_FSYNC, 0xfff, 1 },
		{ CS42L77_ASP2_ENABLE, ASP2_FSYNC_BCLK_EN, ASP2_FSYNC_BCLK_EN },
	};
	u32 val;
	int i, ret;

	gpiod_set_value_cansleep(cs->reset, 1);
	msleep(10);
	gpiod_set_value_cansleep(cs->reset, 0);
	msleep(2);

	ret = cs42l77_ctrl_set(cs, CTRL_RUN);
	if (ret)
		return ret;
	/* status reads 0 here, so it cannot be polled */
	msleep(75);
	ret = cs42l77_ctrl_set(cs, CTRL_RUN_DONE);
	if (ret)
		return ret;

	ret = cs42l77_read(cs, CS42L77_DEVID, &val);
	if (ret)
		return dev_err_probe(dev, ret, "reading DEVID\n");
	if (val != DEVID_CS42L77)
		return dev_err_probe(dev, -ENODEV, "unexpected DEVID %#x\n", val);

	/* The control PLL carries the bus across the core clock switch */
	ret = cs42l77_ctrl_set(cs, CTRL_PLL_KEEP);
	if (ret)
		return ret;
	msleep(400); /* its lock status only shows after a later bus access */

	ret = cs42l77_update(cs, CS42L77_CLK_SRC, CLK_SRC_MASK, 0);
	ret = ret ?: cs42l77_write(cs, CS42L77_CORE_CLK, CORE_CLK_CCM);
	if (ret)
		return dev_err_probe(dev, ret, "core clock switch\n");
	msleep(10);

	ret = cs42l77_read(cs, CS42L77_DEVID, &val);
	if (ret || val != DEVID_CS42L77)
		return dev_err_probe(dev, ret ?: -EIO,
				     "no core clock, is MCLK running?\n");

	ret = cs42l77_ctrl_set(cs, CTRL_RUN | BIT(2));
	if (ret)
		return ret;
	msleep(50);
	ret = cs42l77_ctrl_set(cs, CTRL_RUN_DONE | BIT(2));
	if (ret)
		return ret;
	msleep(10);

	/* Times out while the load keeps the bus busy; OTP_DONE is the check */
	cs42l77_write(cs, CS42L77_OTP_INIT, 1);
	for (i = 0; i < 20; i++) {
		ret = cs42l77_read(cs, CS42L77_OTP_STATUS, &val);
		if (ret || (val & OTP_DONE))
			break;
		usleep_range(1000, 2000);
	}
	if (ret || !(val & OTP_DONE))
		return dev_err_probe(dev, ret ?: -ETIMEDOUT, "OTP load\n");

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		ret = cs42l77_update(cs, seq[i].reg, seq[i].mask, seq[i].val);
		if (ret)
			return dev_err_probe(dev, ret, "writing %#x\n", seq[i].reg);
	}

	ret = cs42l77_read(cs, CS42L77_ASP2_ENABLE, &val);
	if (ret)
		return ret;
	if ((val & ASP2_FSYNC_BCLK_EN) != ASP2_FSYNC_BCLK_EN)
		return dev_err_probe(dev, -EIO, "ASP2 did not start\n");

	dev_info(dev, "ASP2 running\n");
	return 0;
}

static int cs42l77_component_probe(struct snd_soc_component *component)
{
	/* ASP2 stays on from here; gating it per stream would save idle power */
	return cs42l77_start(snd_soc_component_get_drvdata(component));
}

static const struct snd_soc_component_driver cs42l77_component = {
	.probe = cs42l77_component_probe,
};

static int cs42l77_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct cs42l77 *cs;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->tx = devm_kzalloc(dev, 64, GFP_KERNEL);
	cs->rx = devm_kzalloc(dev, 64, GFP_KERNEL);
	if (!cs->tx || !cs->rx)
		return -ENOMEM;
	cs->spi = spi;
	spi_set_drvdata(spi, cs);

	cs->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->reset))
		return PTR_ERR(cs->reset);

	return devm_snd_soc_register_component(dev, &cs42l77_component, NULL, 0);
}

static const struct of_device_id cs42l77_of_match[] = {
	{ .compatible = "cirrus,cs42l77" },
	{}
};
MODULE_DEVICE_TABLE(of, cs42l77_of_match);

static const struct spi_device_id cs42l77_spi_id[] = {
	{ "cs42l77" },
	{}
};
MODULE_DEVICE_TABLE(spi, cs42l77_spi_id);

static struct spi_driver cs42l77_spi_driver = {
	.driver = {
		.name = "cs42l77",
		.of_match_table = cs42l77_of_match,
	},
	.probe = cs42l77_spi_probe,
	.id_table = cs42l77_spi_id,
};
module_spi_driver(cs42l77_spi_driver);

MODULE_DESCRIPTION("CS42L77 codec driver, speaker bus master only");
MODULE_LICENSE("GPL");
