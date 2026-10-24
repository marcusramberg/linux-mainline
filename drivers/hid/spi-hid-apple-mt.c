// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple iPhone multitouch digitizer: HBPP firmware upload over SPI, then
 * Apple HID-over-SPI transport.
 *
 * Boot after drivers/input/touchscreen/apple_z2.c, transport after the Asahi
 * Linux spi-hid-apple driver (Copyright The Asahi Linux Contributors).
 */

#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hid.h>
#include <linux/interrupt.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

/* HIDSPI config shipped with the firmware: Transfer Length, Padding Length,
 * Inter Stage Us, Max Input Report Length. */
#define MT_PKT_LEN		512
#define MT_PKT_DATA		(MT_PKT_LEN - 10)
#define MT_STATUS_LEN		4
#define MT_INTER_STAGE_US	200
#define MT_MAX_REPORT		4096
#define MT_DESC_MAX		512
#define MT_MAX_IFACES		4
#define MT_DEV_INFO		0xd0
#define MT_READ_PACKET		0x20
#define MT_WRITE_PACKET		0x40
#define MT_WAIT			msecs_to_jiffies(1000)

#define HBPP_FW_MAGIC		0x5746325A
#define HBPP_CMD_BLOB		0x3001
#define HBPP_INIT_PAYLOAD	0
#define HBPP_SEND_BLOB		1
#define HBPP_SEND_CALIBRATION	2
#define HBPP_CAL_PROP		"apple,z2-cal-blob"

static const u8 mt_booted[4] = { 0xa0, 0x80, 0x00, 0x00 };
static const u8 mt_status_ok[4] = { 0xac, 0x27, 0x68, 0xd5 };

struct mt_msg_hdr {
	u8 type;
	u8 report;
	u8 device;
	u8 id;
	__le16 rsplen;
	__le16 length;
} __packed;

struct mt_packet {
	u8 flags;
	u8 device;
	__le16 offset;
	__le16 remain;
	__le16 length;
	u8 data[MT_PKT_DATA];
	__le16 crc16;
} __packed;

struct hbpp_blob_hdr {
	__le16 cmd;
	__le16 len;
	__le32 addr;
	__le16 checksum;
} __packed;

struct mt_iface {
	struct apple_mt *mt;
	struct hid_device *hid;
	u8 hid_desc[MT_DESC_MAX];
	u32 hid_desc_len;
	u32 id;
	u32 max_input_report_len;
	u8 name[32];
	u8 reply_buf[MT_DESC_MAX];
	u32 reply_len;
	bool ready;
};

struct mt_report {
	u8 *buf;
	u32 length;
	u32 offset;
	u8 device;
	u8 flags;
};

struct apple_mt {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	const char *fw_name;

	struct completion boot_irq;
	bool hbpp_done;

	wait_queue_head_t wait;
	struct mutex tx_lock;
	struct spi_message rx_msg, tx_msg;
	struct spi_transfer rx_xfer, tx_xfer, status_xfer;
	u8 *rx_buf, *tx_buf, *status_buf;
	u8 hbpp_ack[2];
	u8 hbpp_ack_rx[2];
	unsigned int hbpp_blobs;

	struct mt_iface ifaces[MT_MAX_IFACES];
	struct mt_report report;
	u32 num_devices;
	u16 vendor_id, product_id, version_number;
	u8 serial[32];
	u8 msg_id;
	bool booted;
};

/* ---- HBPP upload ------------------------------------------------------ */

static const u8 *hbpp_build_cal_blob(struct apple_mt *mt, u32 address,
				     size_t *size)
{
	struct device *dev = &mt->spi->dev;
	struct hbpp_blob_hdr *hdr;
	u8 *cal_data;
	int cal_size, i, err;
	u16 checksum_hdr = 0;
	u32 checksum = 0;

	if (!device_property_present(dev, HBPP_CAL_PROP))
		return NULL;
	cal_size = device_property_count_u8(dev, HBPP_CAL_PROP);
	if (cal_size < 0)
		return ERR_PTR(cal_size);

	*size = sizeof(*hdr) + cal_size + sizeof(__le32);
	u8 *blob __free(kfree) = kzalloc(*size, GFP_KERNEL);
	if (!blob)
		return ERR_PTR(-ENOMEM);

	hdr = (struct hbpp_blob_hdr *)blob;
	hdr->cmd = cpu_to_le16(HBPP_CMD_BLOB);
	hdr->len = cpu_to_le16(round_up(cal_size, 4) / 4);
	hdr->addr = cpu_to_le32(address);
	for (i = 2; i < 8; i++)
		checksum_hdr += blob[i];
	hdr->checksum = cpu_to_le16(checksum_hdr);

	cal_data = blob + sizeof(*hdr);
	err = device_property_read_u8_array(dev, HBPP_CAL_PROP, cal_data, cal_size);
	if (err)
		return ERR_PTR(err);
	for (i = 0; i < cal_size; i++)
		checksum += cal_data[i];
	put_unaligned_le32(checksum, cal_data + cal_size);

	return no_free_ptr(blob);
}

static int hbpp_send_blob(struct apple_mt *mt, const u8 *data, u32 size,
			  bool init)
{
	struct spi_transfer xfers[2] = {
		{ .tx_buf = data, .len = size, .bits_per_word = init ? 8 : 16 },
		{ .tx_buf = mt->hbpp_ack, .rx_buf = mt->hbpp_ack_rx, .len = 2 },
	};
	int err;

	reinit_completion(&mt->boot_irq);
	err = spi_sync_transfer(mt->spi, xfers, 2);
	if (err)
		return err;
	if (mt->hbpp_blobs++ < 4 || mt->hbpp_ack_rx[0] || mt->hbpp_ack_rx[1])
		dev_info(&mt->spi->dev, "HBPP blob %u (%u bytes) ack %2ph\n",
			 mt->hbpp_blobs, size, mt->hbpp_ack_rx);
	/* The ack irq is not reliable; the bootloader is happy regardless. */
	wait_for_completion_timeout(&mt->boot_irq, msecs_to_jiffies(20));
	return 0;
}

static int hbpp_upload(struct apple_mt *mt)
{
	struct device *dev = &mt->spi->dev;
	const struct firmware *fw __free(firmware) = NULL;
	size_t idx = 8, size;
	u32 cmd, addr;
	int err;

	err = request_firmware(&fw, mt->fw_name, dev);
	if (err)
		return dev_err_probe(dev, err, "firmware %s\n", mt->fw_name);
	if (fw->size < 8 || get_unaligned_le32(fw->data) != HBPP_FW_MAGIC ||
	    get_unaligned_le32(fw->data + 4) != 1)
		return -EINVAL;

	while (idx < fw->size) {
		if (fw->size - idx < 8)
			return -EINVAL;
		cmd = get_unaligned_le32(fw->data + idx);
		idx += 4;
		switch (cmd) {
		case HBPP_INIT_PAYLOAD:
		case HBPP_SEND_BLOB:
			size = get_unaligned_le32(fw->data + idx);
			idx += 4;
			if (fw->size - idx < size)
				return -EINVAL;
			err = hbpp_send_blob(mt, fw->data + idx, size,
					     cmd == HBPP_INIT_PAYLOAD);
			if (err)
				return err;
			idx += size;
			break;
		case HBPP_SEND_CALIBRATION:
			addr = get_unaligned_le32(fw->data + idx);
			idx += 4;
			const u8 *cal __free(kfree) = hbpp_build_cal_blob(mt, addr, &size);
			if (IS_ERR(cal))
				return PTR_ERR(cal);
			if (cal) {
				err = hbpp_send_blob(mt, cal, size, false);
				if (err)
					return err;
			}
			break;
		default:
			return -EINVAL;
		}
		idx = round_up(idx, 4);
	}
	return 0;
}

/* ---- HID-over-SPI transport ------------------------------------------- */

static int mt_request(struct apple_mt *mt, u8 target, u8 type, u8 report,
		      u8 device, u16 resp_len, const u8 *buf, size_t len)
{
	struct mt_packet *pkt = (struct mt_packet *)mt->tx_buf;
	struct mt_msg_hdr *hdr = (struct mt_msg_hdr *)pkt->data;
	u16 crc;
	int err;

	if (len > sizeof(pkt->data) - sizeof(*hdr) - 2)
		return -EINVAL;

	err = mutex_lock_interruptible(&mt->tx_lock);
	if (err)
		return err;

	memset(pkt, 0, sizeof(*pkt));
	pkt->flags = MT_WRITE_PACKET;
	pkt->device = target;
	pkt->length = cpu_to_le16(sizeof(*hdr) + len + 2);
	hdr->type = type;
	hdr->report = report;
	hdr->device = device;
	hdr->id = mt->msg_id++;
	hdr->rsplen = cpu_to_le16(resp_len);
	hdr->length = cpu_to_le16(len);
	if (len)
		memcpy(pkt->data + sizeof(*hdr), buf, len);
	crc = crc16(0, pkt->data, sizeof(*hdr) + len);
	put_unaligned_le16(crc, pkt->data + sizeof(*hdr) + len);
	pkt->crc16 = cpu_to_le16(crc16(0, mt->tx_buf, offsetof(struct mt_packet, crc16)));

	memset(mt->status_buf, 0, MT_STATUS_LEN);
	err = spi_sync(mt->spi, &mt->tx_msg);
	if (memcmp(mt->status_buf, mt_status_ok, MT_STATUS_LEN))
		dev_warn_ratelimited(&mt->spi->dev, "status %4ph\n", mt->status_buf);
	mutex_unlock(&mt->tx_lock);
	return err ? err : (int)len;
}

static struct mt_iface *mt_get_iface(struct apple_mt *mt, u32 id)
{
	return id < MT_MAX_IFACES ? &mt->ifaces[id] : NULL;
}

static int mt_ll_start(struct hid_device *hdev) { return 0; }
static void mt_ll_stop(struct hid_device *hdev) {}

static int mt_ll_open(struct hid_device *hdev)
{
	struct mt_iface *iface = hdev->driver_data;

	iface->ready = true;
	return 0;
}

static void mt_ll_close(struct hid_device *hdev)
{
	struct mt_iface *iface = hdev->driver_data;

	iface->ready = false;
}

static int mt_ll_parse(struct hid_device *hdev)
{
	struct mt_iface *iface = hdev->driver_data;

	return hid_parse_report(hdev, iface->hid_desc, iface->hid_desc_len);
}

static int mt_ll_raw_request(struct hid_device *hdev, unsigned char reportnum,
			     __u8 *buf, size_t len, unsigned char rtype,
			     int reqtype)
{
	struct mt_iface *iface = hdev->driver_data;
	struct apple_mt *mt = iface->mt;
	int ret;

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		if (rtype != HID_FEATURE_REPORT)
			return -EINVAL;
		iface->reply_len = 0;
		ret = mt_request(mt, iface->id, 0x32, reportnum, 0, len, NULL, 0);
		if (ret < 0)
			return ret;
		ret = wait_event_interruptible_timeout(mt->wait, iface->reply_len, MT_WAIT);
		if (ret == 0)
			return -ETIMEDOUT;
		if (ret < 0)
			return ret;
		memcpy(buf, iface->reply_buf, min_t(size_t, len, iface->reply_len));
		return iface->reply_len;
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return mt_request(mt, iface->id, 0x52, reportnum, 0, 2, buf, len);
	default:
		return -EIO;
	}
}

static int mt_ll_output_report(struct hid_device *hdev, __u8 *buf, size_t len)
{
	struct mt_iface *iface = hdev->driver_data;

	return mt_request(iface->mt, iface->id, 0x51, iface->id, 0, 0, buf, len);
}

static const struct hid_ll_driver mt_hid_ll = {
	.start = mt_ll_start,
	.stop = mt_ll_stop,
	.open = mt_ll_open,
	.close = mt_ll_close,
	.parse = mt_ll_parse,
	.raw_request = mt_ll_raw_request,
	.output_report = mt_ll_output_report,
	.max_buffer_size = MT_MAX_REPORT,
};

static int mt_register_hid(struct apple_mt *mt, struct mt_iface *iface)
{
	struct hid_device *hid;
	int ret;

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return PTR_ERR(hid);

	snprintf(hid->name, sizeof(hid->name), "Apple SPI %s", iface->name);
	snprintf(hid->phys, sizeof(hid->phys), "%s (%x)", dev_name(&mt->spi->dev), iface->id);
	strscpy(hid->uniq, mt->serial, sizeof(hid->uniq));
	hid->ll_driver = &mt_hid_ll;
	hid->bus = BUS_SPI;
	hid->vendor = mt->vendor_id;
	hid->product = mt->product_id;
	hid->version = mt->version_number;
	hid->dev.parent = &mt->spi->dev;
	hid->driver_data = iface;

	ret = hid_add_device(hid);
	if (ret) {
		hid_destroy_device(hid);
		return ret;
	}
	iface->hid = hid;
	return 0;
}

struct mt_device_info {
	__le16 u0[2];
	__le16 num_devices;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_number;
	__le16 vendor_str[2];
	__le16 product_str[2];
	__le16 serial_str[2];
} __packed;

struct mt_iface_info {
	u8 u_0;
	u8 interface_num;
	u8 u_2, u_3, u_4;
	u8 country_code;
	__le16 max_input_report_len;
	__le16 max_output_report_len;
	__le16 max_control_report_len;
	__le16 name_offset;
	__le16 name_length;
} __packed;

static void mt_process_response(struct apple_mt *mt, u32 device,
				struct mt_msg_hdr *hdr, u8 *pl, size_t len)
{
	struct device *dev = &mt->spi->dev;
	struct mt_iface *iface;
	u16 off, slen;

	if (hdr->type == 0x20 && hdr->report == 0x01) {
		struct mt_device_info *info = (void *)pl;

		if (mt->vendor_id || len < sizeof(*info))
			return;
		mt->num_devices = min_t(u32, le16_to_cpu(info->num_devices), MT_MAX_IFACES);
		mt->vendor_id = le16_to_cpu(info->vendor_id);
		mt->product_id = le16_to_cpu(info->product_id);
		mt->version_number = le16_to_cpu(info->version_number);
		off = le16_to_cpu(info->serial_str[0]);
		slen = le16_to_cpu(info->serial_str[1]);
		if (off < len && slen <= len - off && slen < sizeof(mt->serial)) {
			memcpy(mt->serial, pl + off, slen);
			mt->serial[slen] = '\0';
		}
		dev_info(dev, "device %04x:%04x v%u, %u interfaces\n", mt->vendor_id,
			 mt->product_id, mt->version_number, mt->num_devices);
		wake_up_interruptible(&mt->wait);
		return;
	}

	if (hdr->type == 0x20 && hdr->report == 0x02) {
		struct mt_iface_info *info = (void *)pl;

		iface = mt_get_iface(mt, hdr->device);
		if (!iface || iface->max_input_report_len || len < sizeof(*info))
			return;
		iface->max_input_report_len = le16_to_cpu(info->max_input_report_len);
		off = le16_to_cpu(info->name_offset);
		slen = le16_to_cpu(info->name_length);
		if (off < len && slen <= len - off && slen < sizeof(iface->name)) {
			memcpy(iface->name, pl + off, slen);
			iface->name[slen] = '\0';
		}
		dev_info(dev, "interface %u: %s\n", hdr->device, iface->name);
		wake_up_interruptible(&mt->wait);
		return;
	}

	if (hdr->type == 0x20 && hdr->report == 0x10) {
		iface = mt_get_iface(mt, hdr->device);
		if (!iface || iface->hid_desc_len || len > MT_DESC_MAX)
			return;
		memcpy(iface->hid_desc, pl, len);
		iface->hid_desc_len = len;
		wake_up_interruptible(&mt->wait);
		return;
	}

	if (hdr->type == 0x32) {
		iface = mt_get_iface(mt, device);
		if (!iface || len < 1 || len > sizeof(iface->reply_buf))
			return;
		memcpy(iface->reply_buf, pl, len);
		iface->reply_len = len;
		wake_up_interruptible(&mt->wait);
		return;
	}

	dev_dbg(dev, "unhandled response type %02x rep %02x dev %02x len %zu\n",
		hdr->type, hdr->report, hdr->device, len);
}

static void mt_process_message(struct apple_mt *mt, u8 *data, size_t length,
			       u8 device, u8 flags)
{
	struct mt_msg_hdr *hdr = (struct mt_msg_hdr *)data;
	struct mt_iface *iface;
	size_t pl_len;
	u8 *pl;

	if (crc16(0, data, length - 2) != get_unaligned_le16(data + length - 2)) {
		dev_warn_ratelimited(&mt->spi->dev, "message crc mismatch\n");
		return;
	}
	pl_len = le16_to_cpu(hdr->length);
	if (!pl_len || pl_len + sizeof(*hdr) + 2 > length)
		return;
	pl = data + sizeof(*hdr);

	if (flags == MT_WRITE_PACKET) {
		mt_process_response(mt, device, hdr, pl, pl_len);
		return;
	}
	if (flags != MT_READ_PACKET || hdr->type != 0x10)
		return;
	iface = mt_get_iface(mt, device);
	if (iface && iface->hid && iface->ready)
		hid_input_report(iface->hid, HID_INPUT_REPORT, pl, pl_len, 1);
	else
		dev_dbg(&mt->spi->dev, "input report for idle iface %u len %zu\n",
			device, pl_len);
}

static void mt_assemble(struct apple_mt *mt, struct mt_packet *pkt)
{
	struct mt_report *rep = &mt->report;
	size_t length = le16_to_cpu(pkt->length);
	size_t remain = le16_to_cpu(pkt->remain);
	size_t offset = le16_to_cpu(pkt->offset);

	if (offset + length + remain > MT_MAX_REPORT)
		return;
	if (pkt->device != rep->device || pkt->flags != rep->flags || offset != rep->offset) {
		rep->offset = rep->length = 0;
		rep->device = rep->flags = 0;
	}

	if (offset == 0) {
		memcpy(rep->buf, pkt->data, length);
		rep->offset = length;
		rep->length = length + remain;
		rep->device = pkt->device;
		rep->flags = pkt->flags;
	} else if (offset == rep->offset && offset + length + remain == rep->length) {
		memcpy(rep->buf + offset, pkt->data, length);
		rep->offset += length;
		if (rep->offset == rep->length) {
			mt_process_message(mt, rep->buf, rep->length, rep->device, rep->flags);
			rep->offset = rep->length = 0;
			rep->device = rep->flags = 0;
		}
	}
}

static void mt_read_packet(struct apple_mt *mt)
{
	struct mt_packet *pkt = (struct mt_packet *)mt->rx_buf;
	size_t length;
	int err;

	err = spi_sync(mt->spi, &mt->rx_msg);
	if (err) {
		dev_warn(&mt->spi->dev, "rx failed: %d\n", err);
		return;
	}
	if (crc16(0, mt->rx_buf, offsetof(struct mt_packet, crc16)) != le16_to_cpu(pkt->crc16)) {
		dev_warn_ratelimited(&mt->spi->dev, "packet crc mismatch: %16ph\n", mt->rx_buf);
		return;
	}
	length = le16_to_cpu(pkt->length);
	if (length < sizeof(struct mt_msg_hdr) + 2) {
		if (length == sizeof(mt_booted) && !memcmp(pkt->data, mt_booted, length)) {
			if (!mt->booted) {
				mt->booted = true;
				wake_up_interruptible(&mt->wait);
			}
		} else {
			dev_info_ratelimited(&mt->spi->dev, "short packet, raw %16ph\n", mt->rx_buf);
		}
		return;
	}
	if (length > sizeof(pkt->data))
		return;
	if (pkt->offset == 0 && pkt->remain == 0)
		mt_process_message(mt, pkt->data, length, pkt->device, pkt->flags);
	else
		mt_assemble(mt, pkt);
}

static irqreturn_t mt_irq(int irq, void *data)
{
	struct apple_mt *mt = data;

	if (!mt->hbpp_done)
		complete(&mt->boot_irq);
	else
		mt_read_packet(mt);
	return IRQ_HANDLED;
}

static int mt_boot(struct apple_mt *mt)
{
	struct device *dev = &mt->spi->dev;
	int err, i;

	mt->hbpp_done = false;
	mt->booted = false;
	reinit_completion(&mt->boot_irq);
	enable_irq(mt->spi->irq);
	gpiod_set_value_cansleep(mt->reset_gpio, 0);
	if (!wait_for_completion_timeout(&mt->boot_irq, msecs_to_jiffies(20)))
		dev_warn(dev, "no HBPP ready irq after reset, uploading anyway\n");

	err = hbpp_upload(mt);
	if (err) {
		dev_err(dev, "HBPP upload failed: %d\n", err);
		goto fail;
	}
	dev_info(dev, "HBPP upload done\n");
	mt->hbpp_done = true;

	err = wait_event_interruptible_timeout(mt->wait, mt->booted, MT_WAIT);
	if (err == 0)
		err = -ENODEV;
	if (err < 0) {
		dev_err(dev, "no boot message after upload: %d\n", err);
		goto fail;
	}

	mt_request(mt, MT_DEV_INFO, 0x20, 0x01, MT_DEV_INFO, 0, NULL, 0);
	err = wait_event_interruptible_timeout(mt->wait, mt->vendor_id, MT_WAIT);
	if (err == 0)
		err = -ENODEV;
	if (err < 0) {
		dev_err(dev, "no device info: %d\n", err);
		goto fail;
	}

	for (i = 0; i < mt->num_devices; i++) {
		struct mt_iface *iface = &mt->ifaces[i];

		iface->id = i;
		iface->mt = mt;
		mt_request(mt, MT_DEV_INFO, 0x20, 0x02, i, MT_DESC_MAX, NULL, 0);
		wait_event_interruptible_timeout(mt->wait, iface->max_input_report_len, MT_WAIT);
		mt_request(mt, MT_DEV_INFO, 0x20, 0x10, i, MT_DESC_MAX, NULL, 0);
		wait_event_interruptible_timeout(mt->wait, iface->hid_desc_len, MT_WAIT);
		if (!iface->hid_desc_len) {
			dev_warn(dev, "interface %u: no HID descriptor\n", i);
			continue;
		}
		if (!iface->hid && mt_register_hid(mt, iface))
			dev_warn(dev, "interface %u: HID registration failed\n", i);
	}
	return 0;

fail:
	gpiod_set_value_cansleep(mt->reset_gpio, 1);
	disable_irq(mt->spi->irq);
	return err;
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

static int mt_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct apple_mt *mt;
	int err;

	mt = devm_kzalloc(dev, sizeof(*mt), GFP_KERNEL);
	if (!mt)
		return -ENOMEM;
	mt->spi = spi;
	mt->rx_buf = devm_kzalloc(dev, MT_MAX_REPORT, GFP_KERNEL);
	mt->tx_buf = devm_kzalloc(dev, MT_PKT_LEN, GFP_KERNEL);
	mt->status_buf = devm_kzalloc(dev, MT_STATUS_LEN, GFP_KERNEL);
	mt->report.buf = devm_kzalloc(dev, MT_MAX_REPORT, GFP_KERNEL);
	if (!mt->rx_buf || !mt->tx_buf || !mt->status_buf || !mt->report.buf)
		return -ENOMEM;
	mt->hbpp_ack[0] = 0x1a;
	mt->hbpp_ack[1] = 0xa1;
	init_completion(&mt->boot_irq);
	init_waitqueue_head(&mt->wait);
	mutex_init(&mt->tx_lock);
	spi_set_drvdata(spi, mt);

	mt->rx_xfer.rx_buf = mt->rx_buf;
	mt->rx_xfer.len = MT_PKT_LEN;
	spi_message_init_with_transfers(&mt->rx_msg, &mt->rx_xfer, 1);
	mt->tx_xfer.tx_buf = mt->tx_buf;
	mt->tx_xfer.len = MT_PKT_LEN;
	mt->tx_xfer.delay.unit = SPI_DELAY_UNIT_USECS;
	mt->tx_xfer.delay.value = MT_INTER_STAGE_US;
	mt->status_xfer.rx_buf = mt->status_buf;
	mt->status_xfer.len = MT_STATUS_LEN;
	spi_message_init(&mt->tx_msg);
	spi_message_add_tail(&mt->tx_xfer, &mt->tx_msg);
	spi_message_add_tail(&mt->status_xfer, &mt->tx_msg);

	err = device_property_read_string(dev, "firmware-name", &mt->fw_name);
	if (err)
		return dev_err_probe(dev, err, "firmware-name\n");

	/*
	 * ADT: function-power_ldo = pKW4(rBK3, 0x5020000), function-clock_enable =
	 * pKW4(gP00, 0x8000000); the enable value is ORed in. Written once per boot so
	 * a re-bind can test other values written through debugfs.
	 */
	if (!smc_done) {
		err = mt_smc_write(dev, SMC_KEY(rBK3), 0x5020001);
		if (!err)
			err = mt_smc_write(dev, SMC_KEY(gP00), 0x8000001);
		if (err)
			return dev_err_probe(dev, err, "smc power/clock\n");
		smc_done = true;
		msleep(10);
	}

	mt->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(mt->reset_gpio), "reset gpio\n");

	err = devm_request_threaded_irq(dev, spi->irq, NULL, mt_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, "apple-mt", mt);
	if (err)
		return dev_err_probe(dev, err, "irq\n");

	usleep_range(5000, 10000);
	return mt_boot(mt);
}

static void mt_remove(struct spi_device *spi)
{
	struct apple_mt *mt = spi_get_drvdata(spi);
	int i;

	disable_irq(spi->irq);
	for (i = 0; i < MT_MAX_IFACES; i++)
		if (mt->ifaces[i].hid)
			hid_destroy_device(mt->ifaces[i].hid);
	gpiod_set_value_cansleep(mt->reset_gpio, 1);
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
