// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple iPhone multitouch digitizer: HBPP firmware upload over SPI, then
 * Apple HID-over-SPI transport. Touches arrive as input report 0x75.
 *
 * Finger records after drivers/input/touchscreen/apple_z2.c; HBPP loader and
 * HIDSPI exchange after Project Cascadia's apple_hidspi_touch.c.
 */

#include <linux/crc16.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define MT_CAL_FW		"apple/mtfw-d431-cal.bin"

#define HBPP_FW_MAGIC		0x5746325A
#define HBPP_INIT_PAYLOAD	0
#define HBPP_SEND_BLOB		1
#define HBPP_SEND_CALIBRATION	2
#define HBPP_CAL_PROP		"apple,z2-cal-blob"
#define HBPP_CMD_BLOB		0x3001
#define HBPP_CMD_RMW		0x1e33
#define HBPP_CMD_REQ_CAL	0x1f01
#define HBPP_CHUNK		0x3f0
#define HBPP_DATA_OK		0x4bc1
#define HBPP_REG_OK		0x4ad1

#define MT_PAD			4
#define MT_PKT_LEN		512
#define MT_XFER			(MT_PAD + MT_PKT_LEN)
#define MT_BUF_LEN		1100	/* 18 e1 + 14 + HBPP_CHUNK */
#define MT_PKT_MAGIC		0xdeadbeef
#define MT_STATUS_OK		0xd56827ac
#define MT_STATUS_NAK		0x0e4fb139
#define MT_TYPE_DEV_INPUT	0x20
#define MT_MAX_FAILS		20
#define MT_MIN_GAP_US		2000	/* bounds the read rate if the line sticks low */

#define MT_REPORT_TOUCH		0x75
#define MT_FINGERS_OFFSET	32
#define MT_FINGER_LEN		30
#define MT_MAX_SLOTS		16
#define MT_TOUCH_GONE		0
#define MT_TOUCH_STARTED	3
#define MT_TOUCH_MOVED		4
#define MT_TOUCH_LIFTED		7

#define MT_RING_FRAMES		128
#define MT_RING_SLOT		514

struct mt_finger {
	u8 finger;
	u8 state;
	__le16 unknown2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __packed;

struct apple_mt {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	const char *fw_name;
	struct input_dev *input;
	struct touchscreen_properties props;
	int min_x, max_x, min_y, max_y;
	bool inv_x, inv_y;
	DECLARE_BITMAP(down, 256);	/* finger ids reported down */
	DECLARE_BITMAP(prev, 256);	/* finger ids in the previous frame */
	struct task_struct *thread;
	struct completion irq_done;
	u8 *tx, *rx, *last, *pkt;
	unsigned int fails;
	/* debug: ring of raw 0x75 frames, [u16 len][512 bytes] each, exposed in debugfs */
	u8 *ring;
	unsigned int ring_pos;
	struct debugfs_blob_wrapper ring_blob;
	struct dentry *dbg;
};

static bool raw;
module_param(raw, bool, 0644);
MODULE_PARM_DESC(raw, "report the panel's raw positions, for calibration");

static unsigned int release_ms = 60;
module_param(release_ms, uint, 0644);
MODULE_PARM_DESC(release_ms, "let go of fingers after this long without a frame");

static int mt_xfer(struct apple_mt *mt, size_t len)
{
	struct spi_transfer t = { .tx_buf = mt->tx, .rx_buf = mt->rx, .len = len };

	return spi_sync_transfer(mt->spi, &t, 1);
}

/* ---- HBPP upload ------------------------------------------------------ */

static int hbpp_ack(struct apple_mt *mt)
{
	int err;

	mt->tx[0] = 0x1a;
	mt->tx[1] = 0xa1;
	err = mt_xfer(mt, 2);
	return err ? err : get_unaligned_be16(mt->rx);
}

/* One packet of the file (LE halfwords), sent as 16-bit words. */
static int hbpp_send(struct apple_mt *mt, const u8 *pkt, size_t len)
{
	u16 cmd = get_unaligned_le16(pkt);
	int want = cmd == HBPP_CMD_RMW ? HBPP_REG_OK : HBPP_DATA_OK;
	size_t i, off = cmd == HBPP_CMD_BLOB ? 2 : 0;
	int st, tries;

	if (len & 1 || len + off > MT_BUF_LEN)
		return -EINVAL;
	for (tries = 0; tries < 5; tries++) {
		mt->tx[0] = 0x18;
		mt->tx[1] = 0xe1;
		for (i = 0; i < len; i++)
			mt->tx[off + i] = pkt[i ^ 1];
		st = mt_xfer(mt, len + off);
		if (st)
			return st;
		/* the firmware starts within ms of REQ_CAL: the last RMW must beat it */
		if (cmd == HBPP_CMD_BLOB)
			msleep(20);
		else
			usleep_range(200, 400);
		st = hbpp_ack(mt);
		if (cmd == HBPP_CMD_REQ_CAL) {
			dev_info(&mt->spi->dev, "calibration request: %04x\n", st);
			return 0;
		}
		/* the last RMW starts the firmware: its padding (deadbeef) answers */
		if (st == want || st == 0xefbe)
			return 0;
	}
	dev_err(&mt->spi->dev, "HBPP packet %04x: status %04x\n", cmd, st);
	return -EIO;
}

/* A 0x3001 blob (header, csum16, payload, sum32), resent in pieces. */
static int hbpp_blob(struct apple_mt *mt, const u8 *b, size_t len)
{
	u8 *pkt = mt->pkt;
	u32 addr = get_unaligned_le32(b + 4), sum;
	size_t plen = len - 14, off, n, i;
	int err;

	for (off = 0; off < plen; off += n) {
		n = min_t(size_t, plen - off, HBPP_CHUNK);
		put_unaligned_le16(HBPP_CMD_BLOB, pkt);
		put_unaligned_le16((n + 3) / 4, pkt + 2);
		put_unaligned_le32(addr + off, pkt + 4);
		put_unaligned_le16(pkt[2] + pkt[3] + pkt[4] + pkt[5] + pkt[6] + pkt[7], pkt + 8);
		memcpy(pkt + 10, b + 10 + off, n);
		for (sum = 0, i = 0; i < n; i++)
			sum += b[10 + off + i];
		put_unaligned_le32(sum, pkt + 10 + n);
		err = hbpp_send(mt, pkt, 14 + n);
		if (err)
			return err;
	}
	return 0;
}

/* The firmware has no calibration of its own: without this it reads stale SRAM. */
static int hbpp_calibration(struct apple_mt *mt, u32 addr)
{
	struct device *dev = &mt->spi->dev;
	const struct firmware *fw __free(firmware) = NULL;
	const u8 *cal = NULL;
	u32 sum = 0;
	int len, i;

	len = device_property_count_u8(dev, HBPP_CAL_PROP);
	if (len <= 16) {	/* the ADT's "syscfg/MtCl" placeholder is no blob */
		if (firmware_request_nowarn(&fw, MT_CAL_FW, dev)) {
			dev_warn(dev, "no calibration: expect phantom contacts\n");
			return 0;
		}
		cal = fw->data;
		len = fw->size;
	}

	u8 *b __free(kfree) = kzalloc(14 + len, GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	put_unaligned_le16(HBPP_CMD_BLOB, b);
	put_unaligned_le16((len + 3) / 4, b + 2);
	put_unaligned_le32(addr, b + 4);
	put_unaligned_le16(b[2] + b[3] + b[4] + b[5] + b[6] + b[7], b + 8);
	if (cal)
		memcpy(b + 10, cal, len);
	else
		device_property_read_u8_array(dev, HBPP_CAL_PROP, b + 10, len);
	for (i = 0; i < len; i++)
		sum += b[10 + i];
	put_unaligned_le32(sum, b + 10 + len);
	dev_info(dev, "calibration: %d bytes (%s)\n", len, cal ? MT_CAL_FW : HBPP_CAL_PROP);
	return hbpp_blob(mt, b, 14 + len);
}

static int hbpp_upload(struct apple_mt *mt)
{
	struct device *dev = &mt->spi->dev;
	const struct firmware *fw __free(firmware) = NULL;
	size_t idx = 8;
	int err;

	err = request_firmware(&fw, mt->fw_name, dev);
	if (err)
		return err;
	if (fw->size < 8 || get_unaligned_le32(fw->data) != HBPP_FW_MAGIC ||
	    get_unaligned_le32(fw->data + 4) != 1)
		return -EINVAL;

	while (idx + 8 <= fw->size) {
		u32 cmd = get_unaligned_le32(fw->data + idx);
		u32 arg = get_unaligned_le32(fw->data + idx + 4);
		const u8 *p = fw->data + idx + 8;

		idx += 8;
		switch (cmd) {
		case HBPP_INIT_PAYLOAD:
		case HBPP_SEND_BLOB:
			if (arg > fw->size - idx)
				return -EINVAL;
			if (cmd == HBPP_INIT_PAYLOAD)
				err = 0;	/* the D431 firmware has none */
			else if (arg >= 14 && get_unaligned_le16(p) == HBPP_CMD_BLOB)
				err = hbpp_blob(mt, p, arg);
			else
				err = hbpp_send(mt, p, arg);
			idx += arg;
			break;
		case HBPP_SEND_CALIBRATION:
			err = hbpp_calibration(mt, arg);
			break;
		default:
			return -EINVAL;
		}
		if (err)
			return err;
		idx = round_up(idx, 4);
	}
	return 0;
}

/* ---- HID-over-SPI transport ------------------------------------------- */

/*
 * Our filler (0xff) for the device's packet, then the status stage under the
 * same chip select. It must stay 0xff: an all-zero packet has a valid CRC and
 * sends the firmware back to its bootloader.
 */
static bool mt_exchange(struct apple_mt *mt)
{
	struct spi_transfer t[2] = {
		{ .delay = { .value = 300, .unit = SPI_DELAY_UNIT_USECS } },	/* CS Delay */
		{ .tx_buf = mt->tx, .rx_buf = mt->rx, .len = MT_XFER, .cs_change = 1 },
	};
	struct spi_transfer st = {
		.tx_buf = mt->tx, .rx_buf = mt->rx + MT_XFER, .len = MT_PAD + 4,
	};
	const u8 *pk = mt->rx + MT_PAD;
	u16 len;
	bool ok;

	memset(mt->tx, 0xff, MT_XFER);
	if (spi_sync_transfer(mt->spi, t, 2))
		return false;

	len = get_unaligned_le16(pk + 6);
	ok = crc16(0, pk, MT_PKT_LEN - 2) == get_unaligned_le16(pk + MT_PKT_LEN - 2);
	if (!ok && len >= 10 && len <= MT_PKT_LEN - 10) {
		u16 mlen = get_unaligned_le16(pk + 8 + 6);

		ok = mlen + 10 <= len &&
		     crc16(0, pk + 8, 8 + mlen) == get_unaligned_le16(pk + 16 + mlen);
	}
	if (!ok && len == 4 && pk[0] == MT_TYPE_DEV_INPUT)	/* messages carry no CRC */
		ok = true;

	usleep_range(200, 250);			/* Inter Stage Us */
	memset(mt->tx, 0xff, MT_PAD);
	put_unaligned_le32(ok ? MT_STATUS_OK : MT_STATUS_NAK, mt->tx + MT_PAD);
	spi_sync_transfer(mt->spi, &st, 1);
	usleep_range(250, 300);			/* Inter Transaction Us */
	return ok && get_unaligned_le32(mt->rx) == MT_PKT_MAGIC;
}

static bool mt_is_down(const struct mt_finger *f)
{
	return f->state == MT_TOUCH_STARTED || f->state == MT_TOUCH_MOVED;
}

/*
 * Lifted, never reported down and not in the last frame: a tap that fell
 * between reads. A contact seen in the last frame is not one: D431 contacts
 * can go 1 -> 7 -> 0 without ever reaching 3.
 */
static bool mt_is_missed(struct apple_mt *mt, const struct mt_finger *f)
{
	return (f->state == MT_TOUCH_LIFTED || f->state == MT_TOUCH_GONE) &&
	       !test_bit(f->finger, mt->down) && !test_bit(f->finger, mt->prev);
}

static void mt_report_finger(struct apple_mt *mt, const struct mt_finger *f, bool down)
{
	struct input_dev *input = mt->input;
	int slot = input_mt_get_slot_by_key(input, f->finger);
	int x, y;

	if (slot < 0)
		return;
	input_mt_slot(input, slot);
	if (!input_mt_report_slot_state(input, MT_TOOL_FINGER, down))
		return;
	x = (s16)le16_to_cpu(f->abs_x);
	y = (s16)le16_to_cpu(f->abs_y);
	if (!raw) {
		/* inverted axes turn around min..max: the panel's edges are not at 0 */
		if (mt->inv_x)
			x = mt->min_x + mt->max_x - x;
		if (mt->inv_y)
			y = mt->min_y + mt->max_y - y;
		x = clamp(x, mt->min_x, mt->max_x);
		y = clamp(y, mt->min_y, mt->max_y);
	}
	touchscreen_report_pos(input, &mt->props, max(x, 0), max(y, 0), true);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, le16_to_cpu(f->touch_major));
	input_report_abs(input, ABS_MT_TOUCH_MINOR, le16_to_cpu(f->touch_minor));
	input_report_abs(input, ABS_MT_ORIENTATION, (s16)le16_to_cpu(f->orientation));
}

/* evdev drops repeated values: forget lifted positions, or a second tap on the spot has none. */
static void mt_sync_frame(struct apple_mt *mt)
{
	struct input_mt *imt = mt->input->mt;
	int i;

	input_mt_sync_frame(mt->input);
	input_sync(mt->input);
	for (i = 0; i < imt->num_slots; i++) {
		struct input_mt_slot *slot = &imt->slots[i];

		if (input_mt_get_value(slot, ABS_MT_TRACKING_ID) < 0) {
			input_mt_set_value(slot, ABS_MT_POSITION_X, INT_MIN);
			input_mt_set_value(slot, ABS_MT_POSITION_Y, INT_MIN);
		}
	}
}

/* Report 0x75: finger bytes at 16, count at 22, 30-byte records at 32, then a trailer. */
static void mt_parse_touches(struct apple_mt *mt, const u8 *msg, size_t len)
{
	const struct mt_finger *f = (const void *)(msg + MT_FINGERS_OFFSET);
	bool missed = false;
	int i, n;

	if (len < MT_FINGERS_OFFSET)
		return;
	n = min_t(int, get_unaligned_le32(msg + 16) / MT_FINGER_LEN,
		  (len - MT_FINGERS_OFFSET) / MT_FINGER_LEN);

	for (i = 0; i < n; i++)
		missed |= mt_is_missed(mt, &f[i]);
	if (missed) {
		for (i = 0; i < n; i++)
			mt_report_finger(mt, &f[i], mt_is_down(&f[i]) || mt_is_missed(mt, &f[i]));
		mt_sync_frame(mt);
	}
	bitmap_zero(mt->prev, 256);
	for (i = 0; i < n; i++)
		set_bit(f[i].finger, mt->prev);

	for (i = 0; i < n; i++) {
		bool down = mt_is_down(&f[i]);

		mt_report_finger(mt, &f[i], down);
		if (down)
			set_bit(f[i].finger, mt->down);
		else
			clear_bit(f[i].finger, mt->down);
	}
	mt_sync_frame(mt);
}

/* Returns true if a touch frame was reported. */
static bool mt_read(struct apple_mt *mt)
{
	const u8 *pk = mt->rx + MT_PAD, *m = pk + 8;
	u8 *slot;
	u16 len, mlen;

	if (!mt_exchange(mt)) {
		mt->fails++;
		return false;
	}
	mt->fails = 0;
	if (!memcmp(mt->last, mt->rx, MT_XFER))
		return false;
	memcpy(mt->last, mt->rx, MT_XFER);

	len = get_unaligned_le16(pk + 6);
	if (pk[0] != MT_TYPE_DEV_INPUT || len < 10)
		return false;
	mlen = get_unaligned_le16(m + 6);
	if (m[0] != 0x10 || m[1] != MT_REPORT_TOUCH || mlen + 10 > len)
		return false;

	slot = mt->ring + (mt->ring_pos++ % MT_RING_FRAMES) * MT_RING_SLOT;
	put_unaligned_le16(mlen, slot);
	memcpy(slot + 2, m + 8, mlen);
	mt_parse_touches(mt, m + 8, mlen);
	return true;
}

/* ---- boot and thread --------------------------------------------------- */

/* Level-triggered: stays masked until the thread has read, so it cannot storm. */
static irqreturn_t mt_irq(int irq, void *data)
{
	struct apple_mt *mt = data;

	disable_irq_nosync(irq);
	complete(&mt->irq_done);
	return IRQ_HANDLED;
}

static void mt_wait(struct apple_mt *mt, unsigned int ms)
{
	int irq = mt->spi->irq;

	usleep_range(MT_MIN_GAP_US, MT_MIN_GAP_US + 500);
	reinit_completion(&mt->irq_done);
	enable_irq(irq);
	if (!wait_for_completion_timeout(&mt->irq_done, msecs_to_jiffies(ms))) {
		disable_irq(irq);
		if (try_wait_for_completion(&mt->irq_done))
			enable_irq(irq);
	}
}

static int mt_boot(struct apple_mt *mt)
{
	struct device *dev = &mt->spi->dev;
	int err;

	gpiod_set_value_cansleep(mt->reset_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(mt->reset_gpio, 0);
	msleep(20);

	err = hbpp_upload(mt);
	if (err) {
		dev_err(dev, "HBPP upload failed: %d\n", err);
		return err;
	}
	msleep(40);
	memset(mt->last, 0, MT_XFER);
	bitmap_zero(mt->down, 256);
	bitmap_zero(mt->prev, 256);
	mt->fails = 0;
	dev_info(dev, "firmware running\n");
	return 0;
}

static int mt_thread(void *data)
{
	struct apple_mt *mt = data;
	unsigned long last_frame = jiffies;

	while (!kthread_should_stop()) {
		if (mt_boot(mt)) {
			msleep_interruptible(5000);
			continue;
		}
		while (!kthread_should_stop() && mt->fails < MT_MAX_FAILS) {
			bool idle;

			if (mt_read(mt)) {
				int more = 8;

				while (more-- && mt_read(mt))
					;
				last_frame = jiffies;
			} else if (!bitmap_empty(mt->down, 256) &&
				   time_after(jiffies, last_frame + msecs_to_jiffies(release_ms))) {
				/* the firmware streams while touched, so the lift was lost */
				mt_sync_frame(mt);
				bitmap_zero(mt->down, 256);
				bitmap_zero(mt->prev, 256);
			}
			idle = bitmap_empty(mt->down, 256);
			mt_wait(mt, idle ? 1000 : release_ms);
		}
		if (!kthread_should_stop())
			dev_warn(&mt->spi->dev, "firmware not answering, reloading\n");
	}
	return 0;
}

static bool smc_done;

static int mt_smc_write(struct device *dev, smc_key key, u32 val)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "apple,smc", 0);
	struct platform_device *pdev;
	struct apple_smc *smc;
	int err;

	if (!np)
		return -ENODEV;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	smc = dev_get_drvdata(&pdev->dev);
	put_device(&pdev->dev);
	if (!smc)
		return -EPROBE_DEFER;
	err = apple_smc_write_u32(smc, key, val);
	if (err)
		dev_err(dev, "SMC write %p4ch = %#x failed: %d\n", &key, val, err);
	return err;
}

static int mt_input_init(struct apple_mt *mt)
{
	struct input_dev *input = devm_input_allocate_device(&mt->spi->dev);
	int err;

	if (!input)
		return -ENOMEM;
	input->name = "Apple iPhone touchscreen";
	input->phys = "spi-hid-apple-mt/input0";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x05ac;
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 0x1fff, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 0x3fff, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, -32768, 32767, 0, 0);
	touchscreen_parse_properties(input, true, &mt->props);
	mt->min_x = input_abs_get_min(input, ABS_MT_POSITION_X);
	mt->max_x = input_abs_get_max(input, ABS_MT_POSITION_X);
	mt->min_y = input_abs_get_min(input, ABS_MT_POSITION_Y);
	mt->max_y = input_abs_get_max(input, ABS_MT_POSITION_Y);
	mt->inv_x = mt->props.invert_x;
	mt->inv_y = mt->props.invert_y;
	mt->props.invert_x = false;
	mt->props.invert_y = false;
	err = input_mt_init_slots(input, MT_MAX_SLOTS, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (err)
		return err;
	mt->input = input;
	return input_register_device(input);
}

static int mt_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_mt *mt;
	int err;

	mt = devm_kzalloc(dev, sizeof(*mt), GFP_KERNEL);
	if (!mt)
		return -ENOMEM;
	mt->spi = spi;
	mt->tx = devm_kzalloc(dev, MT_BUF_LEN, GFP_KERNEL);
	mt->rx = devm_kzalloc(dev, MT_BUF_LEN, GFP_KERNEL);
	mt->last = devm_kzalloc(dev, MT_BUF_LEN, GFP_KERNEL);
	mt->pkt = devm_kzalloc(dev, MT_BUF_LEN, GFP_KERNEL);
	mt->ring = devm_kzalloc(dev, MT_RING_FRAMES * MT_RING_SLOT, GFP_KERNEL);
	if (!mt->tx || !mt->rx || !mt->last || !mt->pkt || !mt->ring)
		return -ENOMEM;
	init_completion(&mt->irq_done);
	spi_set_drvdata(spi, mt);

	err = device_property_read_string(dev, "firmware-name", &mt->fw_name);
	if (err)
		return dev_err_probe(dev, err, "firmware-name\n");

	/*
	 * ADT: function-clock_enable = pKW4(gP00, 0x8000000); the enable value is
	 * ORed in. The LDO (rBK3) is "Always Powered": iOS only enables the clock.
	 */
	if (!smc_done) {
		err = mt_smc_write(dev, SMC_KEY(gP00), 0x8000001);
		if (err)
			return dev_err_probe(dev, err, "smc clock\n");
		smc_done = true;
		msleep(10);
	}

	mt->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(mt->reset_gpio), "reset gpio\n");

	err = devm_request_irq(dev, spi->irq, mt_irq, IRQF_NO_AUTOEN, "apple-mt", mt);
	if (err)
		return dev_err_probe(dev, err, "irq\n");

	err = mt_input_init(mt);
	if (err)
		return dev_err_probe(dev, err, "input device\n");

	mt->ring_blob.data = mt->ring;
	mt->ring_blob.size = MT_RING_FRAMES * MT_RING_SLOT;
	mt->dbg = debugfs_create_dir("apple-mt", NULL);
	debugfs_create_blob("frames", 0400, mt->dbg, &mt->ring_blob);
	debugfs_create_u32("ring_pos", 0400, mt->dbg, &mt->ring_pos);

	/* the status stage is a second message: no suspend between the two */
	err = pm_runtime_resume_and_get(spi->controller->dev.parent);
	if (err)
		return err;

	mt->thread = kthread_run(mt_thread, mt, "apple-mt");
	if (IS_ERR(mt->thread)) {
		pm_runtime_put(spi->controller->dev.parent);
		return PTR_ERR(mt->thread);
	}
	return 0;
}

static void mt_remove(struct spi_device *spi)
{
	struct apple_mt *mt = spi_get_drvdata(spi);

	debugfs_remove(mt->dbg);
	kthread_stop(mt->thread);
	gpiod_set_value_cansleep(mt->reset_gpio, 1);
	pm_runtime_put(spi->controller->dev.parent);
}

static const struct of_device_id mt_of_match[] = {
	{ .compatible = "apple,d431-touchscreen" },
	{}
};
MODULE_DEVICE_TABLE(of, mt_of_match);

static struct spi_driver mt_driver = {
	.driver = {
		.name = "spi-hid-apple-mt",
		.of_match_table = mt_of_match,
	},
	.probe = mt_probe,
	.remove = mt_remove,
};
module_spi_driver(mt_driver);

MODULE_DESCRIPTION("Apple HBPP-booted HID-over-SPI multitouch driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MT_CAL_FW);
