// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6358 PMIC fuel gauge ADC: battery current through the sense
 * resistor. The bootloader starts the gauge; this only latches and reads it.
 */

#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define MT6358_FGADC_CON1		0xd0a
#define  FG_SW_READ_PRE			BIT(0)
#define  FG_SW_CLEAR			BIT(3)
#define  FG_LATCHDATA_ST		BIT(15)
#define MT6358_FGADC_CUR_CON0		0xd8a

/* one LSB is 381.47 uA with a 10 mOhm sense resistor */
#define MT6358_FGADC_LSB_NA_10MOHM	381470

struct mt6358_fgadc {
	struct regmap *regmap;
	struct mutex lock;
	u32 shunt_uohm;
};

static int mt6358_fgadc_read_current(struct mt6358_fgadc *fg, int *val)
{
	unsigned int reg, cur;
	int ret;

	guard(mutex)(&fg->lock);

	ret = regmap_write(fg->regmap, MT6358_FGADC_CON1, FG_SW_READ_PRE);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(fg->regmap, MT6358_FGADC_CON1, reg,
				       reg & FG_LATCHDATA_ST, 100, 10000);
	if (ret)
		return ret;

	ret = regmap_read(fg->regmap, MT6358_FGADC_CUR_CON0, &cur);
	if (ret)
		return ret;

	regmap_update_bits(fg->regmap, MT6358_FGADC_CON1,
			   FG_SW_CLEAR | FG_SW_READ_PRE, FG_SW_CLEAR);
	ret = regmap_read_poll_timeout(fg->regmap, MT6358_FGADC_CON1, reg,
				       !(reg & FG_LATCHDATA_ST), 100, 10000);
	regmap_clear_bits(fg->regmap, MT6358_FGADC_CON1, FG_SW_CLEAR);
	if (ret)
		return ret;

	/* positive while charging */
	*val = sign_extend32(cur, 15);
	return 0;
}

static int mt6358_fgadc_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct mt6358_fgadc *fg = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = mt6358_fgadc_read_current(fg, val);
		return ret ?: IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* mA per LSB: 381470 nA * 10000 uOhm / shunt */
		*val = MT6358_FGADC_LSB_NA_10MOHM / 10;
		*val2 = fg->shunt_uohm * 10;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info mt6358_fgadc_info = {
	.read_raw = mt6358_fgadc_read_raw,
};

static const struct iio_chan_spec mt6358_fgadc_channels[] = {
	{
		.type = IIO_CURRENT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int mt6358_fgadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct mt6358_fgadc *fg;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*fg));
	if (!indio_dev)
		return -ENOMEM;
	fg = iio_priv(indio_dev);

	/* pwrap's regmap: the parent of the MT6397 MFD */
	fg->regmap = dev_get_regmap(dev->parent->parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(dev, -ENODEV, "no regmap\n");

	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				       &fg->shunt_uohm);
	if (ret || !fg->shunt_uohm)
		return dev_err_probe(dev, -EINVAL, "invalid shunt resistor\n");

	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;

	indio_dev->name = "mt6358-fgadc";
	indio_dev->info = &mt6358_fgadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mt6358_fgadc_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6358_fgadc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id mt6358_fgadc_of_match[] = {
	{ .compatible = "mediatek,mt6358-fgadc" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6358_fgadc_of_match);

static struct platform_driver mt6358_fgadc_driver = {
	.driver = {
		.name = "mt6358-fgadc",
		.of_match_table = mt6358_fgadc_of_match,
	},
	.probe = mt6358_fgadc_probe,
};
module_platform_driver(mt6358_fgadc_driver);

MODULE_DESCRIPTION("MediaTek MT6358 PMIC fuel gauge ADC driver");
MODULE_LICENSE("GPL");
