// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth for the CONNSYS block of MediaTek MT6768 (CONNAC1, A-die MT6631)
 *
 * The CONNSYS power sequencer boots the subsystem MCU, which Bluetooth
 * shares with Wi-Fi, GPS and FM; this driver patches and initialises it
 * with WMT commands over BTIF. Both WMT and HCI travel in STP frames,
 * mandatory mode: no sequence numbers, acks or CRC.
 */

#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pwrseq/consumer.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

#define STP_HDR_SIZE			4
#define STP_TLR_SIZE			2
#define STP_MAX_LEN			2000	/* the vendor parser's limit */
#define STP_TYPE_BT			0
#define STP_TYPE_WMT			4
#define STP_TYPE_ASSERT			5
#define STP_TYPE_INFO			6

#define WMT_PATCH			0x01
#define WMT_TEST			0x02
#define WMT_STRAP_CONF			0x05
#define WMT_FUNC_CTRL			0x06
#define WMT_RESET			0x07
#define WMT_COEX			0x10
#define WMT_RF_CAL			0x14

#define WMT_HDR_SIZE			4
#define WMT_EVT_MAX			64
#define WMT_TIMEOUT			msecs_to_jiffies(2000)

#define FW_PATCH	"mediatek/soc1_0_patch_mcu_1a_1_hdr.bin"

/* header of the MCU patch; a 2-byte checksum follows */
struct consys_patch_hdr {
	u8 date[16];
	u8 platform[4];
	__le16 hw_ver;
	__le16 sw_ver;
	__le32 addr;	/* 0x0001c011, the low byte is a flag */
	__le16 csum;
} __packed;

struct btmtkconnsys_dev {
	struct hci_dev *hdev;
	struct serdev_device *serdev;
	struct hci_uart hu;	/* only for h4_recv_buf() */
	struct pwrseq_desc *pwrseq;

	struct mutex tx_lock;	/* one STP frame at a time */

	/* receive state */
	u8 stp_hdr[STP_HDR_SIZE];
	unsigned int stp_hlen;
	unsigned int stp_len;	/* payload + trailer still to come */
	unsigned int stp_pos;	/* payload received so far */
	u8 stp_type;
	struct sk_buff *rx_skb;
	u8 wmt_buf[WMT_EVT_MAX];
	bool asserted;

	spinlock_t wmt_lock;	/* wmt_op, wmt_evt, wmt_len */
	u8 wmt_op;		/* awaited reply, 0 for none */
	u8 wmt_evt[WMT_EVT_MAX];
	unsigned int wmt_len;
	struct completion wmt_done;
};

static int consys_write(struct btmtkconnsys_dev *bdev, const void *data,
			size_t len)
{
	ssize_t ret;

	ret = serdev_device_write(bdev->serdev, data, len, WMT_TIMEOUT);
	if (ret < 0)
		return ret;
	return ret == len ? 0 : -ETIMEDOUT;
}

static int consys_write_stp(struct btmtkconnsys_dev *bdev, u8 type,
			    const void *data, size_t len)
{
	u8 hdr[STP_HDR_SIZE] = {
		0x80, type << 4 | len >> 8, len & 0xff, 0,
	};
	static const u8 tlr[STP_TLR_SIZE];
	int ret;

	if (len > STP_MAX_LEN)
		return -EINVAL;

	mutex_lock(&bdev->tx_lock);
	ret = consys_write(bdev, hdr, sizeof(hdr));
	if (!ret)
		ret = consys_write(bdev, data, len);
	if (!ret)
		ret = consys_write(bdev, tlr, sizeof(tlr));
	mutex_unlock(&bdev->tx_lock);

	return ret;
}

/*
 * Sends a WMT command and waits for its successful reply, of which up to
 * @evt_len bytes are copied to @evt. Returns the reply's length.
 */
static int consys_wmt(struct btmtkconnsys_dev *bdev, const u8 *cmd,
		      size_t len, u8 *evt, size_t evt_len)
{
	int ret;

	spin_lock_irq(&bdev->wmt_lock);
	bdev->wmt_op = cmd[1];
	reinit_completion(&bdev->wmt_done);
	spin_unlock_irq(&bdev->wmt_lock);

	ret = consys_write_stp(bdev, STP_TYPE_WMT, cmd, len);
	if (!ret && !wait_for_completion_timeout(&bdev->wmt_done, WMT_TIMEOUT))
		ret = -ETIMEDOUT;

	spin_lock_irq(&bdev->wmt_lock);
	bdev->wmt_op = 0;
	spin_unlock_irq(&bdev->wmt_lock);

	if (ret) {
		bt_dev_err(bdev->hdev, "WMT %02x: %d", cmd[1], ret);
		return ret;
	}

	/* 02 <opcode> <len16> <status> ... */
	if (bdev->wmt_len < WMT_HDR_SIZE + 1 || bdev->wmt_evt[4]) {
		bt_dev_err(bdev->hdev, "WMT %02x: bad reply %*ph", cmd[1],
			   (int)min(bdev->wmt_len, 8U), bdev->wmt_evt);
		return -EIO;
	}

	if (evt)
		memcpy(evt, bdev->wmt_evt, min_t(size_t, evt_len,
						 bdev->wmt_len));
	return bdev->wmt_len;
}

static int consys_wmt_cmd(struct btmtkconnsys_dev *bdev, u8 op,
			  const u8 *data, u8 len)
{
	u8 cmd[WMT_HDR_SIZE + 8] = { 0x01, op, len };
	int ret;

	if (WARN_ON(len > sizeof(cmd) - WMT_HDR_SIZE))
		return -EINVAL;
	memcpy(cmd + WMT_HDR_SIZE, data, len);
	ret = consys_wmt(bdev, cmd, WMT_HDR_SIZE + len, NULL, 0);
	return ret < 0 ? ret : 0;
}

#define WMT_CMD(bdev, op, ...)						\
	consys_wmt_cmd(bdev, op, (const u8[]){ __VA_ARGS__ },		\
		       sizeof((const u8[]){ __VA_ARGS__ }))

static int consys_load_patch(struct btmtkconnsys_dev *bdev)
{
	const struct consys_patch_hdr *hdr;
	const struct firmware *fw;
	u8 cmd[21] = { 0x01, WMT_PATCH, 0x11, 0x00, 0x00 };
	__le32 pda[3] = {};
	size_t len;
	int ret;

	ret = request_firmware(&fw, FW_PATCH, &bdev->serdev->dev);
	if (ret)
		return ret;

	ret = -EINVAL;
	if (fw->size <= sizeof(*hdr))
		goto out;
	hdr = (const void *)fw->data;
	len = fw->size - sizeof(*hdr);

	/* download address, size, scramble key, configuration */
	put_unaligned_le32(le32_to_cpu(hdr->addr) & ~0xff, cmd + 5);
	put_unaligned_le32(len, cmd + 9);
	put_unaligned_le32(0x0002022f, cmd + 17);
	ret = consys_wmt(bdev, cmd, sizeof(cmd), NULL, 0);
	if (ret < 0)
		goto out;

	/* the PDA engine takes the rest raw, not in STP frames */
	pda[0] = cpu_to_le32(len + sizeof(pda));
	mutex_lock(&bdev->tx_lock);
	ret = consys_write(bdev, pda, sizeof(pda));
	if (!ret)
		ret = consys_write(bdev, fw->data + sizeof(*hdr), len);
	mutex_unlock(&bdev->tx_lock);
	if (ret)
		goto out;

	/* starts the patched firmware */
	ret = WMT_CMD(bdev, WMT_RESET, 0x04);
	if (!ret)
		bt_dev_info(bdev->hdev, "MCU patch %.16s", hdr->date);
out:
	release_firmware(fw);
	return ret;
}

/* Same order as the vendor stack's WMT init, minus what this board's config skips */
static int btmtkconnsys_setup(struct hci_dev *hdev)
{
	struct btmtkconnsys_dev *bdev = hci_get_drvdata(hdev);
	u8 evt[9];
	int ret;

	ret = consys_load_patch(bdev);
	if (ret)
		return ret;

	ret = WMT_CMD(bdev, WMT_TEST, 0x11, 0x68, 0x67, 0x00, 0x00); /* chip id */
	if (ret)
		return ret;

	ret = consys_wmt(bdev, (const u8[]){ 0x01, WMT_TEST, 0x01, 0x00, 0x12 },
			 5, evt, sizeof(evt));
	if (ret < 0)
		return ret;
	if (ret >= sizeof(evt))
		bt_dev_info(hdev, "A-die MT%04x", get_unaligned_le16(evt + 7));

	/* BT TX power tracks Wi-Fi's TSSI, target 3776 */
	ret = WMT_CMD(bdev, WMT_TEST, 0x10, 0x03, 0xc0, 0x0e);
	if (!ret)
		ret = WMT_CMD(bdev, WMT_RF_CAL, 0x01);
	if (!ret)
		ret = WMT_CMD(bdev, WMT_COEX, 0x1e, 0x00, 0x00, 0x00);
	if (!ret)
		ret = WMT_CMD(bdev, WMT_COEX, 0x1f, 0x00, 0x00, 0x00);
	if (!ret)
		ret = WMT_CMD(bdev, WMT_COEX, 0x20, 0x00, 0x00, 0x00);
	if (!ret)
		ret = WMT_CMD(bdev, WMT_STRAP_CONF, 0x02, 0x02); /* FM on STP */
	if (!ret)
		ret = WMT_CMD(bdev, WMT_FUNC_CTRL, 0x00, 0x01);	/* BT on */
	return ret;
}

static int btmtkconnsys_shutdown(struct hci_dev *hdev)
{
	return WMT_CMD(hci_get_drvdata(hdev), WMT_FUNC_CTRL, 0x00, 0x00);
}

/* closing and opening again reboots the MCU */
static void btmtkconnsys_reset(struct hci_dev *hdev)
{
	if (!test_bit(HCI_INIT, &hdev->flags))
		hci_reset_dev(hdev);
}

/* one STP frame may carry several WMT events */
static void consys_recv_wmt(struct btmtkconnsys_dev *bdev)
{
	unsigned int len = min_t(unsigned int, bdev->stp_pos,
				 sizeof(bdev->wmt_buf));
	unsigned int pos, n;
	unsigned long flags;
	const u8 *e;

	for (pos = 0; pos + WMT_HDR_SIZE <= len; pos += n) {
		e = bdev->wmt_buf + pos;
		n = WMT_HDR_SIZE + get_unaligned_le16(e + 2);

		spin_lock_irqsave(&bdev->wmt_lock, flags);
		if (e[0] == 0x02 && e[1] == bdev->wmt_op) {
			/* only the start of long events is kept */
			bdev->wmt_len = min(n, len - pos);
			memcpy(bdev->wmt_evt, e, bdev->wmt_len);
			bdev->wmt_op = 0;
			complete(&bdev->wmt_done);
		} else {
			bt_dev_dbg(bdev->hdev, "unexpected WMT %*ph",
				   (int)min(len - pos, 8U), e);
		}
		spin_unlock_irqrestore(&bdev->wmt_lock, flags);
	}
}

static const struct h4_recv_pkt consys_recv_pkts[] = {
	{ H4_RECV_ACL,   .recv = hci_recv_frame },
	{ H4_RECV_SCO,   .recv = hci_recv_frame },
	{ H4_RECV_EVENT, .recv = hci_recv_frame },
	{ H4_RECV_ISO,   .recv = hci_recv_frame },
};

static void consys_recv_payload(struct btmtkconnsys_dev *bdev,
				const u8 *data, size_t count)
{
	switch (bdev->stp_type) {
	case STP_TYPE_BT:
		bdev->rx_skb = h4_recv_buf(&bdev->hu, bdev->rx_skb, data, count,
					   consys_recv_pkts,
					   ARRAY_SIZE(consys_recv_pkts));
		if (IS_ERR(bdev->rx_skb)) {
			bt_dev_err(bdev->hdev, "bad H4 frame (%ld)",
				   PTR_ERR(bdev->rx_skb));
			bdev->rx_skb = NULL;
		}
		break;
	case STP_TYPE_WMT:
		if (bdev->stp_pos < sizeof(bdev->wmt_buf))
			memcpy(bdev->wmt_buf + bdev->stp_pos, data,
			       min(count, sizeof(bdev->wmt_buf) -
					  bdev->stp_pos));
		break;
	case STP_TYPE_ASSERT:
		/* a coredump follows the first frame, which says why */
		if (!bdev->stp_pos && !bdev->asserted) {
			bdev->asserted = true;
			bt_dev_err(bdev->hdev, "MCU assert: %.*s",
				   (int)min_t(size_t, count, 128), data);
			btmtkconnsys_reset(bdev->hdev);
		}
		break;
	}
	bdev->stp_pos += count;
}

/* drop the 0x80 of a bad header and rescan what followed it */
static void consys_stp_resync(struct btmtkconnsys_dev *bdev)
{
	u8 *h = bdev->stp_hdr;
	unsigned int i;

	for (i = 1; i < STP_HDR_SIZE && h[i] != 0x80; i++)
		;
	memmove(h, h + i, STP_HDR_SIZE - i);
	bdev->stp_hlen = STP_HDR_SIZE - i;
}

static size_t btmtkconnsys_receive_buf(struct serdev_device *serdev,
				       const u8 *data, size_t count)
{
	struct btmtkconnsys_dev *bdev = serdev_device_get_drvdata(serdev);
	u8 *h = bdev->stp_hdr;
	size_t left = count;

	while (left) {
		size_t n, payload;

		if (bdev->stp_hlen < STP_HDR_SIZE) {
			if (!bdev->stp_hlen && *data != 0x80) {
				data++;
				left--;
				continue;
			}
			h[bdev->stp_hlen++] = *data++;
			left--;
			if (bdev->stp_hlen < STP_HDR_SIZE)
				continue;

			n = (h[1] & 0xf) << 8 | h[2];
			if (n >= STP_MAX_LEN) {
				consys_stp_resync(bdev);
				continue;
			}
			bdev->stp_type = (h[1] >> 4) & 0x7;
			bdev->stp_pos = 0;
			/* only firmware messages carry a trailer when empty */
			if (!n && bdev->stp_type != STP_TYPE_ASSERT &&
			    bdev->stp_type != STP_TYPE_INFO) {
				bdev->stp_hlen = 0;
				continue;
			}
			bdev->stp_len = n + STP_TLR_SIZE;
			continue;
		}

		n = min(left, (size_t)bdev->stp_len);
		payload = bdev->stp_len > STP_TLR_SIZE ?
			  min(n, (size_t)bdev->stp_len - STP_TLR_SIZE) : 0;
		if (payload)
			consys_recv_payload(bdev, data, payload);

		data += n;
		left -= n;
		bdev->stp_len -= n;
		if (bdev->stp_len)
			continue;

		bdev->stp_hlen = 0;
		if (bdev->stp_type == STP_TYPE_WMT)
			consys_recv_wmt(bdev);
	}

	return count;
}

static const struct serdev_device_ops btmtkconnsys_ops = {
	.receive_buf = btmtkconnsys_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int btmtkconnsys_open(struct hci_dev *hdev)
{
	struct btmtkconnsys_dev *bdev = hci_get_drvdata(hdev);
	int ret;

	ret = pwrseq_power_on(bdev->pwrseq);
	if (ret)
		return ret;

	bdev->stp_hlen = 0;
	bdev->asserted = false;
	ret = serdev_device_open(bdev->serdev);
	if (ret) {
		pwrseq_power_off(bdev->pwrseq);
		return ret;
	}
	/* BTIF has no RTS/CTS */
	serdev_device_set_flow_control(bdev->serdev, false);

	return 0;
}

static int btmtkconnsys_close(struct hci_dev *hdev)
{
	struct btmtkconnsys_dev *bdev = hci_get_drvdata(hdev);

	serdev_device_close(bdev->serdev);
	kfree_skb(bdev->rx_skb);
	bdev->rx_skb = NULL;

	return pwrseq_power_off(bdev->pwrseq);
}

static int btmtkconnsys_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtkconnsys_dev *bdev = hci_get_drvdata(hdev);
	int ret;

	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);
	ret = consys_write_stp(bdev, STP_TYPE_BT, skb->data, skb->len);
	if (ret)
		return ret;

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);
	return 0;
}

static int btmtkconnsys_set_bdaddr(struct hci_dev *hdev,
				   const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;

	skb = __hci_cmd_sync(hdev, 0xfc1a, sizeof(*bdaddr), bdaddr,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	kfree_skb(skb);

	return 0;
}

static int btmtkconnsys_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct btmtkconnsys_dev *bdev;
	struct hci_dev *hdev;
	int ret;

	bdev = devm_kzalloc(dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;
	bdev->serdev = serdev;
	mutex_init(&bdev->tx_lock);
	spin_lock_init(&bdev->wmt_lock);
	init_completion(&bdev->wmt_done);

	bdev->pwrseq = devm_pwrseq_get(dev, "bluetooth");
	if (IS_ERR(bdev->pwrseq))
		return dev_err_probe(dev, PTR_ERR(bdev->pwrseq),
				     "no CONNSYS power sequencer\n");

	serdev_device_set_drvdata(serdev, bdev);
	serdev_device_set_client_ops(serdev, &btmtkconnsys_ops);

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;
	bdev->hdev = hdev;
	bdev->hu.hdev = hdev;
	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, bdev);
	SET_HCIDEV_DEV(hdev, dev);
	hdev->open = btmtkconnsys_open;
	hdev->close = btmtkconnsys_close;
	hdev->setup = btmtkconnsys_setup;
	hdev->shutdown = btmtkconnsys_shutdown;
	hdev->reset = btmtkconnsys_reset;
	hdev->send = btmtkconnsys_send_frame;
	hdev->set_bdaddr = btmtkconnsys_set_bdaddr;
	/* every unit has the chip default 00:00:46:76:63:01 */
	hci_set_quirk(hdev, HCI_QUIRK_USE_BDADDR_PROPERTY);
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	hci_set_quirk(hdev, HCI_QUIRK_BROKEN_LOCAL_EXT_FEATURES_PAGE_2);

	ret = hci_register_dev(hdev);
	if (ret) {
		hci_free_dev(hdev);
		return ret;
	}

	return 0;
}

static void btmtkconnsys_remove(struct serdev_device *serdev)
{
	struct btmtkconnsys_dev *bdev = serdev_device_get_drvdata(serdev);

	hci_unregister_dev(bdev->hdev);
	hci_free_dev(bdev->hdev);
}

static const struct of_device_id btmtkconnsys_of_match[] = {
	{ .compatible = "mediatek,mt6768-bluetooth" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtkconnsys_of_match);

static struct serdev_device_driver btmtkconnsys_driver = {
	.probe = btmtkconnsys_probe,
	.remove = btmtkconnsys_remove,
	.driver = {
		.name = "btmtkconnsys",
		.of_match_table = btmtkconnsys_of_match,
	},
};
module_serdev_device_driver(btmtkconnsys_driver);

MODULE_DESCRIPTION("MediaTek CONNSYS Bluetooth over BTIF");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(FW_PATCH);
