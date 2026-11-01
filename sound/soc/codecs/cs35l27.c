// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cirrus Logic CS35L27 speaker amplifier (Apple D431 top speaker)
 *
 * Register layout follows the CS35L41 at different addresses, as used by
 * iOS AppleCS35L27Amp. Only the I2S/ASP path, with the 12 MHz MCLK iOS uses
 * as PLL reference.
 *
 * The speaker runs only while d431-speaker-guard, which models the coil from
 * the amp's V/I sense, keeps writing Speaker Guard Limit. The PAC firmware only
 * limits brownout. No IRQ line, so faults are polled.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define CS35L27_DEVID		0x0000
#define CS35L27_PDM_PCM_CFG	0x1c08
#define  PDM_PCM_CFG_PDM	BIT(0)
#define CS35L27_GLOBAL_ENABLE	0x2000
#define CS35L27_POWER_BLOCKS	0x2008
#define  POWER_BLOCKS_IOS	0x53304d0d /* with VMON, IMON */
#define CS35L27_POWER_UP_WA	0x2404
#define  POWER_UP_WA_BIT3	BIT(3)
#define CS35L27_INT_STATUS1	0x2810
#define  INT1_AMP_SHORT		BIT(1)
#define  INT1_BST_FAULT		GENMASK(12, 10) /* VBST error, CAP1/2 short */
#define CS35L27_INT_STATUS3	0x2818
#define  INT3_DC_INPUT		(BIT(26) | BIT(24)) /* level, rising */
#define CS35L27_INT_STATUS4	0x281c
#define  INT4_OVERTEMP		(BIT(6) | BIT(4)) /* level, rising */
#define CS35L27_REFCLK_INPUT	0x3000
#define  REFCLK_OPEN_LOOP	BIT(11)
#define  REFCLK_FREQ		GENMASK(10, 5)
#define  REFCLK_SEL		GENMASK(2, 0)
#define  REFCLK_SEL_MCLK	5
#define  REFCLK_FREQ_12MHZ	0x32
#define CS35L27_GLOBAL_FS	0x3004
#define  GLOBAL_FS_48K		0x03
#define CS35L27_MDSYNC		0x4000
#define CS35L27_NOISE_GATE	0x4404
#define  NOISE_GATE_BIT17	BIT(17)
#define CS35L27_PCM_CTRL	0x5000
#define  PCM_CTRL_MAX		0x100770 /* the ceiling: -18 dB, 0.125 dB a unit */
#define CS35L27_PDM_CTRL	0x5400
#define  PDM_CTRL_IOS		0x1007e8
#define CS35L27_ANALOG_GAIN	0x5800
#define  ANALOG_GAIN_MASK	0x1f1f
#define  ANALOG_GAIN_IOS	0x0f03
#define CS35L27_DACPCM1_INPUT	0x7000
#define  INPUT_ASPRX1		0x8
#define CS35L27_ASP_CH_ENABLES	0x7200
#define  ASP_RX1_EN		BIT(16)
#define  ASP_TX_VI_EN		GENMASK(1, 0)
#define CS35L27_SP_FORMAT	0x7208
#define  SP_FORMAT_MASK		0x751
#define  SP_FMT_TDM		0x440 /* as iOS: TDM1.5, inverted BCLK */
#define CS35L27_SP_ENABLES	0x720c
#define CS35L27_SP_TX_SLOT	0x7210
#define  TX_SLOTS_IOS		0x00c000b0 /* VMON slot 11, IMON slot 12 */
#define CS35L27_SP_RX_SLOT	0x7230
#define  RX1_SLOT		GENMASK(7, 0)
#define  RX1_SLOT_IOS		0x20
#define CS35L27_SP_RX_WIDTH	0x7258
#define  RX_WIDTH_16		0x22

#define CS35L27_ID		0x0035a270

#define CS35L27_VOL_STEPS	84	/* 0.5 dB each, down to -60 dB */
#define CS35L27_VOL_DEFAULT	(CS35L27_VOL_STEPS - 12)
#define CS35L27_FAULT_POLL	msecs_to_jiffies(100)
#define CS35L27_GUARD_TIMEOUT	msecs_to_jiffies(500)

static const struct {
	unsigned int reg, fatal;
} cs35l27_faults[] = {
	{ CS35L27_INT_STATUS1, INT1_AMP_SHORT | INT1_BST_FAULT },
	{ CS35L27_INT_STATUS3, INT3_DC_INPUT },
	{ CS35L27_INT_STATUS4, INT4_OVERTEMP },
};

static bool output = true;
module_param(output, bool, 0644);
MODULE_PARM_DESC(output, "power the speaker on playback, if the guard runs");

struct cs35l27 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct delayed_work fault_work;
	struct mutex volume_lock;
	int volume;
	int guard_limit;
	unsigned long guard_seen;
	bool faulted;
};

static int cs35l27_set_volume(struct cs35l27 *cs)
{
	unsigned int val, readback;
	int ret;

	guard(mutex)(&cs->volume_lock);
	val = PCM_CTRL_MAX -
	      (CS35L27_VOL_STEPS - min(cs->volume, cs->guard_limit)) * 4;
	ret = regmap_write(cs->regmap, CS35L27_PCM_CTRL, val);
	if (ret)
		return ret;
	ret = regmap_read(cs->regmap, CS35L27_PCM_CTRL, &readback);
	if (ret)
		return ret;
	return readback == val ? 0 : -EIO;
}

static int cs35l27_volume_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l27 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->volume;
	return 0;
}

static int cs35l27_volume_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l27 *cs = snd_soc_component_get_drvdata(component);
	long val = ucontrol->value.integer.value[0];
	int ret;

	if (val < 0 || val > CS35L27_VOL_STEPS)
		return -EINVAL;
	if (val == cs->volume)
		return 0;

	cs->volume = val;
	ret = cs35l27_set_volume(cs);
	return ret ?: 1;
}

static int cs35l27_guard_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l27 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->guard_limit;
	return 0;
}

static int cs35l27_guard_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l27 *cs = snd_soc_component_get_drvdata(component);
	long val = ucontrol->value.integer.value[0];
	int ret;

	if (val < 0 || val > CS35L27_VOL_STEPS)
		return -EINVAL;

	WRITE_ONCE(cs->guard_seen, jiffies);
	if (val == cs->guard_limit)
		return 0;

	cs->guard_limit = val;
	ret = cs35l27_set_volume(cs);
	return ret ?: 1;
}

static const DECLARE_TLV_DB_SCALE(cs35l27_volume_tlv, -6000, 50, 0);

static const struct snd_kcontrol_new cs35l27_controls[] = {
	SOC_SINGLE_EXT_TLV("Speaker Playback Volume", SND_SOC_NOPM, 0,
			   CS35L27_VOL_STEPS, 0, cs35l27_volume_get,
			   cs35l27_volume_put, cs35l27_volume_tlv),
	SOC_SINGLE_EXT("Speaker Guard Limit", SND_SOC_NOPM, 0,
		       CS35L27_VOL_STEPS, 0, cs35l27_guard_get,
		       cs35l27_guard_put),
};

static int cs35l27_check_faults(struct cs35l27 *cs)
{
	unsigned int val;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cs35l27_faults); i++) {
		ret = regmap_read(cs->regmap, cs35l27_faults[i].reg, &val);
		if (ret)
			return ret;
		if (val & cs35l27_faults[i].fatal) {
			dev_err(cs->dev, "amp fault: %#x = %#x\n",
				cs35l27_faults[i].reg, val);
			return -EIO;
		}
	}
	return 0;
}

static void cs35l27_update_amp(struct cs35l27 *cs)
{
	bool guarded = cs->guard_limit &&
		       time_before(jiffies, READ_ONCE(cs->guard_seen) +
					    CS35L27_GUARD_TIMEOUT);

	if (!guarded)
		dev_warn_once(cs->dev, "no speaker guard, amp off\n");
	regmap_update_bits(cs->regmap, CS35L27_GLOBAL_ENABLE, BIT(0),
			   output && guarded && !cs->faulted);
}

static void cs35l27_fault_work(struct work_struct *work)
{
	struct cs35l27 *cs = container_of(to_delayed_work(work), struct cs35l27,
					  fault_work);

	/* A fault holds the amp off until the next stream */
	if (cs35l27_check_faults(cs))
		cs->faulted = true;
	cs35l27_update_amp(cs);
	schedule_delayed_work(&cs->fault_work, CS35L27_FAULT_POLL);
}

/* Latched bits may predate this stream; level bits survive the clear */
static int cs35l27_stream_ok(struct cs35l27 *cs)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(cs35l27_faults); i++) {
		ret = regmap_write(cs->regmap, cs35l27_faults[i].reg,
				   cs35l27_faults[i].fatal);
		if (ret)
			return ret;
	}
	ret = cs35l27_set_volume(cs);
	return ret ?: cs35l27_check_faults(cs);
}

static int cs35l27_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct cs35l27 *cs = snd_soc_component_get_drvdata(dai->component);
	int ret;

	ret = regmap_update_bits(cs->regmap, CS35L27_GLOBAL_FS, 0x1f,
				 GLOBAL_FS_48K);
	if (ret)
		return ret;

	regmap_set_bits(cs->regmap, CS35L27_REFCLK_INPUT, REFCLK_OPEN_LOOP);
	regmap_update_bits(cs->regmap, CS35L27_REFCLK_INPUT,
			   REFCLK_FREQ | REFCLK_SEL,
			   FIELD_PREP(REFCLK_FREQ, REFCLK_FREQ_12MHZ) |
			   FIELD_PREP(REFCLK_SEL, REFCLK_SEL_MCLK));
	return regmap_clear_bits(cs->regmap, CS35L27_REFCLK_INPUT,
				 REFCLK_OPEN_LOOP);
}

static int cs35l27_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct cs35l27 *cs = snd_soc_component_get_drvdata(dai->component);

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;

	return regmap_update_bits(cs->regmap, CS35L27_SP_FORMAT, SP_FORMAT_MASK,
				  SP_FMT_TDM);
}

static const struct snd_soc_dai_ops cs35l27_dai_ops = {
	.hw_params = cs35l27_hw_params,
	.set_fmt = cs35l27_set_fmt,
};

static struct snd_soc_dai_driver cs35l27_dai = {
	.name = "cs35l27-asp",
	.playback = {
		.stream_name = "ASP Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "ASP Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs35l27_dai_ops,
};

static int cs35l27_amp_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs35l27 *cs = snd_soc_component_get_drvdata(component);
	int ret;

	if (!SND_SOC_DAPM_EVENT_ON(event)) {
		cancel_delayed_work_sync(&cs->fault_work);
		return regmap_clear_bits(cs->regmap, CS35L27_GLOBAL_ENABLE, BIT(0));
	}

	ret = cs35l27_stream_ok(cs);
	if (ret) {
		dev_err(cs->dev, "not enabling the amp: %d\n", ret);
		return ret;
	}

	cs->faulted = false;
	cs35l27_update_amp(cs);
	schedule_delayed_work(&cs->fault_work, CS35L27_FAULT_POLL);
	return 0;
}

static const struct snd_soc_dapm_widget cs35l27_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASPRX1", "ASP Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs35l27_amp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_AIF_OUT("ASPTX", "ASP Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("VI"),
};

static const struct snd_soc_dapm_route cs35l27_routes[] = {
	{ "Amp", NULL, "ASPRX1" },
	{ "SPK", NULL, "Amp" },
	{ "ASPTX", NULL, "VI" },
};

static const struct snd_soc_component_driver cs35l27_component = {
	.controls = cs35l27_controls,
	.num_controls = ARRAY_SIZE(cs35l27_controls),
	.dapm_widgets = cs35l27_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l27_widgets),
	.dapm_routes = cs35l27_routes,
	.num_dapm_routes = ARRAY_SIZE(cs35l27_routes),
	.endianness = 1,
};

static const struct regmap_config cs35l27_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0xfffc,
	.cache_type = REGCACHE_NONE,
};

static int cs35l27_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cs35l27 *cs;
	unsigned int id;
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->dev = dev;
	cs->volume = CS35L27_VOL_DEFAULT;
	i2c_set_clientdata(client, cs);

	ret = devm_mutex_init(dev, &cs->volume_lock);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &cs->fault_work,
					   cs35l27_fault_work);
	if (ret)
		return ret;

	cs->regmap = devm_regmap_init_i2c(client, &cs35l27_regmap);
	if (IS_ERR(cs->regmap))
		return PTR_ERR(cs->regmap);

	cs->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->reset))
		return PTR_ERR(cs->reset);
	if (cs->reset) {
		usleep_range(2000, 3000);
		gpiod_set_value_cansleep(cs->reset, 0);
		usleep_range(2000, 3000);
	}

	ret = regmap_read(cs->regmap, CS35L27_DEVID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "reading DEVID\n");
	if (id != CS35L27_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected DEVID %#x\n", id);

	regmap_clear_bits(cs->regmap, CS35L27_PDM_PCM_CFG, PDM_PCM_CFG_PDM);
	regmap_set_bits(cs->regmap, CS35L27_POWER_UP_WA, POWER_UP_WA_BIT3);
	regmap_write(cs->regmap, CS35L27_POWER_BLOCKS, POWER_BLOCKS_IOS);
	ret = cs35l27_set_volume(cs);
	if (ret)
		return dev_err_probe(dev, ret, "setting the volume ceiling\n");
	regmap_write(cs->regmap, CS35L27_PDM_CTRL, PDM_CTRL_IOS);
	regmap_update_bits(cs->regmap, CS35L27_ANALOG_GAIN, ANALOG_GAIN_MASK,
			   ANALOG_GAIN_IOS);
	regmap_set_bits(cs->regmap, CS35L27_NOISE_GATE, NOISE_GATE_BIT17);
	regmap_write(cs->regmap, CS35L27_MDSYNC, 1);
	regmap_write(cs->regmap, CS35L27_DACPCM1_INPUT, INPUT_ASPRX1);
	regmap_write(cs->regmap, CS35L27_ASP_CH_ENABLES, ASP_RX1_EN | ASP_TX_VI_EN);
	regmap_write(cs->regmap, CS35L27_SP_TX_SLOT, TX_SLOTS_IOS);
	regmap_write(cs->regmap, CS35L27_SP_ENABLES, 0x3);
	regmap_update_bits(cs->regmap, CS35L27_SP_RX_SLOT, RX1_SLOT, RX1_SLOT_IOS);
	regmap_write(cs->regmap, CS35L27_SP_RX_WIDTH, RX_WIDTH_16);

	return devm_snd_soc_register_component(dev, &cs35l27_component,
					       &cs35l27_dai, 1);
}

static const struct of_device_id cs35l27_of_match[] = {
	{ .compatible = "cirrus,cs35l27" },
	{}
};
MODULE_DEVICE_TABLE(of, cs35l27_of_match);

static struct i2c_driver cs35l27_i2c_driver = {
	.driver = {
		.name = "cs35l27",
		.of_match_table = cs35l27_of_match,
	},
	.probe = cs35l27_i2c_probe,
};
module_i2c_driver(cs35l27_i2c_driver);

MODULE_DESCRIPTION("CS35L27 speaker amplifier driver");
MODULE_LICENSE("GPL");
