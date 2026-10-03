// SPDX-License-Identifier: GPL-2.0-only
/*
 * QST QMC6308 3-axis magnetometer
 *
 * Init sequence and scale follow MediaTek's sensor hub firmware driver for
 * this part.
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/unaligned.h>

#define QMC6308_CHIP_ID			0x00
#define QMC6308_CHIP_ID_VAL		0x80
#define QMC6308_DATA			0x01
#define QMC6308_STATUS			0x09
#define QMC6308_STATUS_DRDY		BIT(0)
#define QMC6308_STATUS_OVFL		BIT(1)
/* OSR2 8, OSR1 8, ODR 10 Hz, continuous */
#define QMC6308_CTRL1			0x0a
#define QMC6308_CTRL1_CONT		0xc3
#define QMC6308_CTRL1_SUSPEND		0xc0
/* set/reset on, +-30 G: 1000 LSB/G */
#define QMC6308_CTRL2			0x0b
#define QMC6308_CTRL2_30G		0x00
/* vendor init value, meaning unknown */
#define QMC6308_REG_0D			0x0d

struct qmc6308_data {
	struct i2c_client *client;
	struct iio_mount_matrix orientation;
};

static int qmc6308_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct qmc6308_data *data = iio_priv(indio_dev);
	u8 buf[QMC6308_STATUS - QMC6308_DATA + 1];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_i2c_block_data(data->client, QMC6308_DATA,
						    sizeof(buf), buf);
		if (ret < 0)
			return ret;
		if (ret != sizeof(buf))
			return -EIO;
		if (buf[QMC6308_STATUS - QMC6308_DATA] & QMC6308_STATUS_OVFL)
			return -ERANGE;

		*val = (s16)get_unaligned_le16(&buf[chan->address * 2]);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 10;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_mount_matrix *
qmc6308_get_mount_matrix(const struct iio_dev *indio_dev,
			 const struct iio_chan_spec *chan)
{
	struct qmc6308_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info qmc6308_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, qmc6308_get_mount_matrix),
	{ }
};

#define QMC6308_CHANNEL(axis, idx) {					\
	.type = IIO_MAGN,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.address = idx,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.ext_info = qmc6308_ext_info,					\
}

static const struct iio_chan_spec qmc6308_channels[] = {
	QMC6308_CHANNEL(X, 0),
	QMC6308_CHANNEL(Y, 1),
	QMC6308_CHANNEL(Z, 2),
};

static const struct iio_info qmc6308_info = {
	.read_raw = qmc6308_read_raw,
};

static void qmc6308_power_off(void *client)
{
	i2c_smbus_write_byte_data(client, QMC6308_CTRL1, QMC6308_CTRL1_SUSPEND);
}

static int qmc6308_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct qmc6308_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;

	ret = iio_read_mount_matrix(dev, &data->orientation);
	if (ret)
		return ret;

	ret = i2c_smbus_read_byte_data(client, QMC6308_CHIP_ID);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");
	if (ret != QMC6308_CHIP_ID_VAL)
		dev_info(dev, "unknown chip ID 0x%02x\n", ret);

	ret = i2c_smbus_write_byte_data(client, QMC6308_REG_0D, 0x40);
	if (ret)
		return ret;
	ret = i2c_smbus_write_byte_data(client, QMC6308_CTRL2,
					QMC6308_CTRL2_30G);
	if (ret)
		return ret;
	ret = i2c_smbus_write_byte_data(client, QMC6308_CTRL1,
					QMC6308_CTRL1_CONT);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, qmc6308_power_off, client);
	if (ret)
		return ret;

	indio_dev->name = "qmc6308";
	indio_dev->info = &qmc6308_info;
	indio_dev->channels = qmc6308_channels;
	indio_dev->num_channels = ARRAY_SIZE(qmc6308_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id qmc6308_id[] = {
	{ "qmc6308" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qmc6308_id);

static const struct of_device_id qmc6308_of_match[] = {
	{ .compatible = "qst,qmc6308" },
	{ }
};
MODULE_DEVICE_TABLE(of, qmc6308_of_match);

static struct i2c_driver qmc6308_driver = {
	.driver = {
		.name = "qmc6308",
		.of_match_table = qmc6308_of_match,
	},
	.probe = qmc6308_probe,
	.id_table = qmc6308_id,
};
module_i2c_driver(qmc6308_driver);

MODULE_DESCRIPTION("QST QMC6308 magnetometer driver");
MODULE_LICENSE("GPL");
