// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech TDDI touchscreen over SPI (FT8725 in the Motorola Moto G17).
 *
 * The touch controller has no flash: after reset it sits in its boot ROM and
 * the host loads the firmware into its program (PRAM) and data (DRAM) memory.
 * Bus protocol, download sequence and report formats follow the vendor
 * focaltech_touch driver.
 */

#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FTS_WRITE		0x00
#define FTS_READ		(0x80 | 0x20)	/* read, data CRC on */
#define FTS_HDR_LEN		4
#define FTS_DUMMY		3
#define FTS_STATUS_ERR		0xa0
#define FTS_RETRIES		3

#define FTS_REG_TOUCH		0x01
#define FTS_REG_CHIP_ID2	0x9f
#define FTS_REG_CHIP_ID		0xa3
#define FTS_REG_FW_VER		0xa6

/* boot ROM commands */
#define FTS_ROM_START		0x55
#define FTS_ROM_READ_ID		0x90
#define FTS_ROM_SET_ADDR	0xad
#define FTS_ROM_WRITE		0xae
#define FTS_ROM_ECC		0xcc
#define FTS_ROM_ECC_READ	0xcd
#define FTS_ROM_ECC_FINISH	0xce
#define FTS_ROM_START_APP	0x08

/* FT8725 */
#define FTS_CHIP_ID		0x87
#define FTS_CHIP_ID2		0x25
#define FTS_INIT_DELAY_MS	8
#define FTS_LEN_COEF		2
#define FTS_ECC_MAX		(128 * 1024)
#define FTS_ECC_OK		0xa5
#define FTS_DRAM_BASE		0xd00000
#define FTS_APP_INFO		0x100
#define FTS_PACKET		(32 * 1024 - 16)
#define FTS_FW_NAME		"focaltech_ts_fw_dijin.bin"

#define FTS_MAX_POINTS		10
#define FTS_TOUCH_LEN		(FTS_MAX_POINTS * 8 + 4)
#define FTS_XFER_MAX		(FTS_HDR_LEN + FTS_DUMMY + FTS_PACKET + 2)

#define FTS_ETYPE_DEFAULT	0x0
#define FTS_ETYPE_V2		0x2
#define FTS_FLAG_UP		1

struct fts_spi {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset;
	struct touchscreen_properties prop;
	u8 *tx, *rx;
	u8 buf[FTS_TOUCH_LEN];
};

static int fts_xfer(struct fts_spi *ts, size_t xlen)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = xlen,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	/* the controller wants CS high for a while between transfers */
	usleep_range(150, 200);
	if (ret)
		return ret;

	return ts->rx[3] & FTS_STATUS_ERR ? -EIO : 0;
}

static int fts_write(struct fts_spi *ts, u8 cmd, const void *data, size_t len)
{
	size_t xlen = FTS_HDR_LEN;
	int i, ret = -EIO;

	ts->tx[0] = cmd;
	ts->tx[1] = FTS_WRITE;
	ts->tx[2] = len >> 8;
	ts->tx[3] = len;
	if (len) {
		memset(ts->tx + FTS_HDR_LEN, 0, FTS_DUMMY);
		memcpy(ts->tx + FTS_HDR_LEN + FTS_DUMMY, data, len);
		xlen += FTS_DUMMY + len;
	}

	for (i = 0; i < FTS_RETRIES && ret; i++)
		ret = fts_xfer(ts, xlen);

	return ret;
}

static int fts_read(struct fts_spi *ts, u8 addr, u8 *data, size_t len)
{
	const size_t dp = FTS_HDR_LEN + FTS_DUMMY;
	const size_t xlen = dp + len + 2;
	int i, ret = -EIO;

	memset(ts->tx, 0, xlen);
	ts->tx[0] = addr;
	ts->tx[1] = FTS_READ;
	ts->tx[2] = len >> 8;
	ts->tx[3] = len;

	for (i = 0; i < FTS_RETRIES; i++) {
		ret = fts_xfer(ts, xlen);
		if (ret)
			continue;
		if (crc_ccitt(0xffff, ts->rx + dp, len) !=
		    get_unaligned_le16(ts->rx + dp + len)) {
			ret = -EBADMSG;
			continue;
		}
		memcpy(data, ts->rx + dp, len);
		return 0;
	}

	return ret;
}

static int fts_enter_boot(struct fts_spi *ts)
{
	u8 id[2];
	int i, j;

	for (i = 0; i < 30; i++) {
		gpiod_set_value_cansleep(ts->reset, 1);
		usleep_range(2000, 3000);
		gpiod_set_value_cansleep(ts->reset, 0);
		mdelay(FTS_INIT_DELAY_MS + i * 2);

		for (j = 0; j < 3; j++) {
			if (fts_write(ts, FTS_ROM_START, NULL, 0))
				continue;
			msleep(FTS_INIT_DELAY_MS);
			if (!fts_read(ts, FTS_ROM_READ_ID, id, 2) &&
			    id[0] == FTS_CHIP_ID && id[1] == FTS_CHIP_ID2)
				return 0;
		}
	}

	return -ENODEV;
}

static int fts_mem_write(struct fts_spi *ts, u32 base, const u8 *buf, u32 len)
{
	u32 off, n;
	u8 addr[3];
	int ret;

	for (off = 0; off < len; off += n) {
		n = min_t(u32, len - off, FTS_PACKET);
		put_unaligned_be24(base + off, addr);
		ret = fts_write(ts, FTS_ROM_SET_ADDR, addr, sizeof(addr)) ?:
		      fts_write(ts, FTS_ROM_WRITE, buf + off, n);
		if (ret)
			return ret;
	}

	return 0;
}

static u16 fts_ecc_host(const u8 *data, u32 len)
{
	u16 ecc = 0;
	u32 i, j;

	for (i = 0; i < len; i += 2) {
		ecc ^= get_unaligned_be16(data + i);
		for (j = 0; j < 16; j++)
			ecc = ecc & 1 ? (ecc >> 1) ^ 0x8408 : ecc >> 1;
	}

	return ecc;
}

/* addresses in the ECC command are relative, as in the vendor driver */
static int fts_ecc_check(struct fts_spi *ts, const u8 *buf, u32 len)
{
	u32 off, n;
	u8 cmd[6], val[2];
	int i, ret;

	for (off = 0; off < len; off += n) {
		n = min_t(u32, len - off, FTS_ECC_MAX);
		put_unaligned_be24(off, cmd);
		put_unaligned_be24(n, cmd + 3);
		ret = fts_write(ts, FTS_ROM_ECC, cmd, sizeof(cmd));
		if (ret)
			return ret;
		msleep(2);

		for (i = 0; i < 100; i++) {
			ret = fts_read(ts, FTS_ROM_ECC_FINISH, val, 1);
			if (ret)
				return ret;
			if (val[0] == FTS_ECC_OK)
				break;
			msleep(1);
		}
		if (i == 100)
			return -ETIMEDOUT;

		ret = fts_read(ts, FTS_ROM_ECC_READ, val, 2);
		if (ret)
			return ret;
		if (get_unaligned_be16(val) != fts_ecc_host(buf + off, n))
			return -EIO;
	}

	return 0;
}

static int fts_download(struct fts_spi *ts, const struct firmware *fw)
{
	const u8 *info = fw->data + FTS_APP_INFO;
	u16 code, dram;
	u32 code_len, dram_len;
	int ret;

	if (fw->size < FTS_APP_INFO + 12)
		return -EINVAL;
	code = get_unaligned_be16(info);
	dram = get_unaligned_be16(info + 8);
	code_len = code * FTS_LEN_COEF;
	dram_len = dram * FTS_LEN_COEF;
	if ((code ^ get_unaligned_be16(info + 2)) != 0xffff || code_len > fw->size)
		return -EINVAL;
	/* no DRAM part is fine */
	if ((dram ^ get_unaligned_be16(info + 10)) != 0xffff ||
	    code_len + dram_len > fw->size)
		dram_len = 0;

	ret = fts_enter_boot(ts);
	if (ret)
		return ret;

	ret = fts_mem_write(ts, 0, fw->data, code_len) ?:
	      fts_ecc_check(ts, fw->data, code_len);
	if (ret)
		return ret;

	if (dram_len) {
		ret = fts_mem_write(ts, FTS_DRAM_BASE, fw->data + code_len, dram_len) ?:
		      fts_ecc_check(ts, fw->data + code_len, dram_len);
		if (ret)
			return ret;
	}

	ret = fts_write(ts, FTS_ROM_START_APP, NULL, 0);
	msleep(10);

	return ret;
}

static void fts_report(struct fts_spi *ts, unsigned int id, u8 flag,
		       unsigned int x, unsigned int y, u8 area)
{
	bool down = flag != FTS_FLAG_UP;

	input_mt_slot(ts->input, id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down);
	if (down) {
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, area ?: 9);
	}
}

static irqreturn_t fts_irq(int irq, void *data)
{
	struct fts_spi *ts = data;
	u8 *b = ts->buf;
	unsigned int i, n, etype;
	int ret;

	ret = fts_read(ts, FTS_REG_TOUCH, b, FTS_TOUCH_LEN);
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "touch read failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	/* 0xff: firmware (re)initialized, 0xef: needs recovery; no touches */
	if ((b[1] == 0xff && b[2] == 0xff) || (b[1] == 0xef && b[2] == 0xef))
		return IRQ_HANDLED;

	etype = b[1] >> 4;
	n = b[1] & 0xf;

	if (etype == FTS_ETYPE_V2) {
		for (i = 0; i < n && i < FTS_MAX_POINTS; i++) {
			u8 *p = b + 4 + 8 * i;
			unsigned int id = p[2] >> 4;

			if (id >= FTS_MAX_POINTS)
				break;
			fts_report(ts, id, p[0] >> 6,
				   (((p[0] & 0xf) << 12) | (p[1] << 4) | (p[4] >> 4)) / 16,
				   (((p[2] & 0xf) << 12) | (p[3] << 4) | (p[4] & 0xf)) / 16,
				   p[5]);
		}
	} else if (etype == FTS_ETYPE_DEFAULT) {
		for (i = 0; i < FTS_MAX_POINTS; i++) {
			u8 *p = b + 2 + 6 * i;
			unsigned int id = p[2] >> 4;

			if (id >= FTS_MAX_POINTS)
				break;
			fts_report(ts, id, p[0] >> 6,
				   ((p[0] & 0xf) << 8) | p[1],
				   ((p[2] & 0xf) << 8) | p[3], p[5]);
		}
	} else {
		dev_dbg(&ts->spi->dev, "event type %#x ignored\n", etype);
		return IRQ_HANDLED;
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static int fts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct firmware *fw;
	struct fts_spi *ts;
	u8 id = 0, id2, ver;
	int i, ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;

	ts->tx = devm_kzalloc(dev, FTS_XFER_MAX, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, FTS_XFER_MAX, GFP_KERNEL);
	if (!ts->tx || !ts->rx)
		return -ENOMEM;

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ts->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset))
		return dev_err_probe(dev, PTR_ERR(ts->reset), "no reset GPIO\n");


	ret = request_firmware(&fw, FTS_FW_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware %s\n", FTS_FW_NAME);
	ret = fts_download(ts, fw);
	release_firmware(fw);
	if (ret)
		return dev_err_probe(dev, ret, "firmware download failed\n");

	for (i = 0; i < 50 && id != FTS_CHIP_ID; i++) {
		msleep(10);
		if (fts_read(ts, FTS_REG_CHIP_ID, &id, 1))
			id = 0;
	}
	if (id != FTS_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "firmware did not start\n");
	if (!fts_read(ts, FTS_REG_CHIP_ID2, &id2, 1) &&
	    !fts_read(ts, FTS_REG_FW_VER, &ver, 1))
		dev_info(dev, "chip %02x%02x, firmware %#x\n", id, id2, ver);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;
	ts->input->name = "FocalTech SPI touchscreen";
	ts->input->id.bustype = BUS_SPI;

	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	ret = input_mt_init_slots(ts->input, FTS_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	return devm_request_threaded_irq(dev, spi->irq, NULL, fts_irq,
					 IRQF_ONESHOT, dev_name(dev), ts);
}

static const struct of_device_id fts_of_match[] = {
	{ .compatible = "focaltech,ft8725-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts_of_match);

static const struct spi_device_id fts_spi_ids[] = {
	{ "ft8725-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, fts_spi_ids);

static struct spi_driver fts_driver = {
	.driver = {
		.name = "focaltech-spi",
		.of_match_table = fts_of_match,
	},
	.id_table = fts_spi_ids,
	.probe = fts_probe,
};
module_spi_driver(fts_driver);

MODULE_FIRMWARE(FTS_FW_NAME);
MODULE_DESCRIPTION("FocalTech TDDI SPI touchscreen driver");
MODULE_LICENSE("GPL");
