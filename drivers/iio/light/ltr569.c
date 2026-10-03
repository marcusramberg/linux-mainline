// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lite-On LTR-569 ambient light and proximity sensor
 *
 * No public datasheet: registers and lux formula follow MediaTek's sensor
 * hub firmware driver for this part.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/unaligned.h>

#define LTR569_ALS_CONTR		0x80
#define LTR569_ALS_CONTR_ON		(BIT(7) | BIT(0))
#define LTR569_ALS_CONTR_GAIN		GENMASK(5, 2)
#define LTR569_PS_CONTR			0x81
#define LTR569_PS_CONTR_ON		(BIT(4) | BIT(1))
#define LTR569_ALS_STATUS		0x88
#define LTR569_ALS_STATUS_INVALID	BIT(6)
#define LTR569_ALS_DATA			0x8b
#define LTR569_PS_DATA			0x9a
#define LTR569_PART_ID			0xb3
#define LTR569_PART_ID_VAL		0x9c

#define LTR569_GAIN_MAX			9	/* 512x */
#define LTR569_LUX_COEF			78

struct ltr569_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 gain;
	u32 near_level;
};

/* Vendor init values, meaning unknown */
static const u8 ltr569_init_regs[][2] = {
	{ 0x85, 0xa7 }, { 0x86, 0xff }, { 0x87, 0x3f }, { 0x98, 0x00 },
	{ 0xbc, 0x00 }, { 0xbf, 0x00 }, { 0xc1, 0x00 }, { 0x7f, 0x53 },
	{ 0x93, 0x52 }, { 0x94, 0xed }, { 0xb6, 0xc5 }, { 0xdb, 0x23 },
	{ 0xe3, 0x57 }, { 0xf9, 0x85 }, { 0xd8, 0x0c },
	/* PS LED, pulses, rate */
	{ 0x82, 0x0c }, { 0x95, 0x03 }, { 0x83, 0x0f }, { 0x96, 0x10 },
	{ 0x84, 0x18 }, { 0x97, 0xe7 },
};

static int ltr569_set_gain(struct ltr569_data *data, u8 gain)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, LTR569_ALS_CONTR,
					LTR569_ALS_CONTR_ON |
					FIELD_PREP(LTR569_ALS_CONTR_GAIN, gain));
	if (ret)
		return ret;

	data->gain = gain;
	return 0;
}

static int ltr569_read_lux(struct ltr569_data *data, int *val, int *val2)
{
	u8 buf[5];
	u16 als;
	u8 gain;
	int ret;

	guard(mutex)(&data->lock);

	ret = i2c_smbus_read_i2c_block_data(data->client, LTR569_ALS_STATUS,
					    sizeof(buf), buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;

	/* the status gain is the one this sample was taken with */
	gain = FIELD_GET(LTR569_ALS_CONTR_GAIN, buf[0]);
	als = get_unaligned_le16(&buf[LTR569_ALS_DATA - LTR569_ALS_STATUS]);

	if ((buf[0] & LTR569_ALS_STATUS_INVALID) || als > 50000) {
		if (data->gain > 0)
			ltr569_set_gain(data, data->gain - 1);
		if (buf[0] & LTR569_ALS_STATUS_INVALID)
			return -EAGAIN;
	} else if (als < 500 && data->gain < LTR569_GAIN_MAX) {
		ltr569_set_gain(data, data->gain + 1);
	}

	*val = als * LTR569_LUX_COEF;
	*val2 = 1 << gain;
	return IIO_VAL_FRACTIONAL;
}

static int ltr569_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ltr569_data *data = iio_priv(indio_dev);

	int ret;

	switch (chan->type) {
	case IIO_LIGHT:
		if (mask != IIO_CHAN_INFO_PROCESSED)
			return -EINVAL;
		return ltr569_read_lux(data, val, val2);
	case IIO_PROXIMITY:
		if (mask != IIO_CHAN_INFO_RAW)
			return -EINVAL;
		ret = i2c_smbus_read_word_data(data->client, LTR569_PS_DATA);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static ssize_t ltr569_read_near_level(struct iio_dev *indio_dev,
				      uintptr_t priv,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct ltr569_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", data->near_level);
}

static const struct iio_chan_spec_ext_info ltr569_ps_ext_info[] = {
	{
		.name = "nearlevel",
		.shared = IIO_SEPARATE,
		.read = ltr569_read_near_level,
	},
	{ }
};

static const struct iio_chan_spec ltr569_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.ext_info = ltr569_ps_ext_info,
	},
};

static const struct iio_info ltr569_info = {
	.read_raw = ltr569_read_raw,
};

static void ltr569_power_off(void *client)
{
	i2c_smbus_write_byte_data(client, LTR569_PS_CONTR, 0);
	i2c_smbus_write_byte_data(client, LTR569_ALS_CONTR, 0);
}

static int ltr569_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ltr569_data *data;
	struct iio_dev *indio_dev;
	int i, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	device_property_read_u32(dev, "proximity-near-level",
				 &data->near_level);
	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	ret = i2c_smbus_read_byte_data(client, LTR569_PART_ID);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read part ID\n");
	if (ret != LTR569_PART_ID_VAL)
		dev_info(dev, "unknown part ID 0x%02x\n", ret);

	for (i = 0; i < ARRAY_SIZE(ltr569_init_regs); i++) {
		ret = i2c_smbus_write_byte_data(client, ltr569_init_regs[i][0],
						ltr569_init_regs[i][1]);
		if (ret)
			return ret;
	}

	ret = ltr569_set_gain(data, LTR569_GAIN_MAX);
	if (ret)
		return ret;

	/* ponytail: PS always on; enable on demand if it costs battery */
	ret = i2c_smbus_write_byte_data(client, LTR569_PS_CONTR,
					LTR569_PS_CONTR_ON);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ltr569_power_off, client);
	if (ret)
		return ret;

	indio_dev->name = "ltr569";
	indio_dev->info = &ltr569_info;
	indio_dev->channels = ltr569_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltr569_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id ltr569_id[] = {
	{ "ltr569" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltr569_id);

static const struct of_device_id ltr569_of_match[] = {
	{ .compatible = "liteon,ltr569" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltr569_of_match);

static struct i2c_driver ltr569_driver = {
	.driver = {
		.name = "ltr569",
		.of_match_table = ltr569_of_match,
	},
	.probe = ltr569_probe,
	.id_table = ltr569_id,
};
module_i2c_driver(ltr569_driver);

MODULE_DESCRIPTION("Lite-On LTR-569 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
