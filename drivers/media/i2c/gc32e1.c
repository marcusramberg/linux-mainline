// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC32E1 32 MP RAW sensor driver
 *
 * Register settings from the Motorola/MediaTek imgsensor driver.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define GC32E1_LANES			4
/* 322.56 Mpix/s of RAW10 on four lanes */
#define GC32E1_LINK_FREQ		403200000
#define GC32E1_MCLK_FREQ		24000000

/* 2x2 binned */
#define GC32E1_WIDTH			3264
#define GC32E1_HEIGHT			2448
#define GC32E1_PIXEL_RATE		345600000
/* twice the line length register */
#define GC32E1_HTS			4276
/* 30 fps */
#define GC32E1_VTS_DEF			2694
#define GC32E1_VTS_MAX			0x3fff

/* no register auto-increment: 16-bit values go one byte at a time */
#define GC32E1_REG_CHIP_ID_H		CCI_REG8(0x03f0)
#define GC32E1_REG_CHIP_ID_L		CCI_REG8(0x03f1)
#define GC32E1_CHIP_ID			0x32e1

/* bit 0 mirror, bit 1 flip; the Bayer order follows */
#define GC32E1_REG_ORIENTATION		CCI_REG8(0x0101)

#define GC32E1_REG_MODE_SELECT		CCI_REG8(0x0100)
#define GC32E1_MODE_STANDBY		0x00
#define GC32E1_MODE_STREAMING		0x01

#define GC32E1_REG_EXPOSURE		0x0202
#define GC32E1_EXPOSURE_MIN		4
#define GC32E1_EXPOSURE_MARGIN		16
#define GC32E1_EXPOSURE_DEF		2424

/* 1024 is 1x */
#define GC32E1_REG_ANALOGUE_GAIN	0x0204
#define GC32E1_AGAIN_MIN		1024
#define GC32E1_AGAIN_MAX		(1024 * 16)

#define GC32E1_REG_VTS			0x0340
#define GC32E1_REG_TEST_PATTERN_EN	CCI_REG8(0x008c)
#define GC32E1_REG_TEST_PATTERN_IDX	CCI_REG8(0x008d)
#define GC32E1_TEST_PATTERN_COLOUR_BARS	0x20

static const char * const gc32e1_supply_names[] = {
	/* power-up order */
	"dovdd",
	"dvdd",
	"avdd",
};

static const struct cci_reg_sequence gc32e1_init_regs[] = {
	{ CCI_REG8(0x0315), 0xc0 },
	{ CCI_REG8(0x0c10), 0x1b },
	{ CCI_REG8(0x01a7), 0x00 },
	{ CCI_REG8(0x01aa), 0x05 },
	{ CCI_REG8(0x01a8), 0x02 },
	{ CCI_REG8(0x0c0d), 0xb4 },
	{ CCI_REG8(0x0314), 0x11 },
	{ CCI_REG8(0x031a), 0x00 },
	{ CCI_REG8(0x01a1), 0x10 },
	{ CCI_REG8(0x01e3), 0x03 },
	{ CCI_REG8(0x0057), 0x03 },
	{ CCI_REG8(0x00a6), 0x06 },
	{ CCI_REG8(0x00d3), 0x30 },
	{ CCI_REG8(0x0087), 0x51 },
	{ CCI_REG8(0x01e2), 0x24 },
	{ CCI_REG8(0x01ea), 0x24 },
	{ CCI_REG8(0x0219), 0x05 },
	{ CCI_REG8(0x0c08), 0x19 },
	{ CCI_REG8(0x0c05), 0xff },
	{ CCI_REG8(0x0c07), 0x14 },
	{ CCI_REG8(0x0c41), 0x0a },
	{ CCI_REG8(0x0c45), 0xdf },
	{ CCI_REG8(0x0e15), 0x58 },
	{ CCI_REG8(0x0e6c), 0x01 },
	{ CCI_REG8(0x0e6d), 0x01 },
	{ CCI_REG8(0x03a2), 0x00 },
	{ CCI_REG8(0x0313), 0x00 },
	{ CCI_REG8(0x0a53), 0x04 },
	{ CCI_REG8(0x0a65), 0x17 },
	{ CCI_REG8(0x0a68), 0x33 },
	{ CCI_REG8(0x0a58), 0x00 },
	{ CCI_REG8(0x0a4f), 0x00 },
	{ CCI_REG8(0x0a66), 0x00 },
	{ CCI_REG8(0x0a7f), 0x07 },
	{ CCI_REG8(0x0a84), 0x0c },
};

static const struct cci_reg_sequence gc32e1_3264x2448_regs[] = {
	{ CCI_REG8(0x031c), 0x60 },
	{ CCI_REG8(0x0315), 0xd3 },
	{ CCI_REG8(0x01a9), 0x38 },
	{ CCI_REG8(0x0c0c), 0x48 },
	{ CCI_REG8(0x0185), 0x40 },
	{ CCI_REG8(0x0c0e), 0x21 },
	{ CCI_REG8(0x0311), 0xb0 },
	{ CCI_REG8(0x05a0), 0x0a },
	{ CCI_REG8(0x0059), 0x11 },
	{ CCI_REG8(0x0084), 0x30 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0202), 0x09 },
	{ CCI_REG8(0x0203), 0x78 },
	{ CCI_REG8(0x0210), 0xa3 },
	{ CCI_REG8(0x0218), 0x03 },
	{ CCI_REG8(0x0226), 0x14 },
	{ CCI_REG8(0x0227), 0x80 },
	{ CCI_REG8(0x0340), 0x0a },
	{ CCI_REG8(0x0341), 0x86 },
	{ CCI_REG8(0x0342), 0x08 },
	{ CCI_REG8(0x0343), 0x5a },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x20 },
	{ CCI_REG8(0x034a), 0x13 },
	{ CCI_REG8(0x034b), 0x40 },
	{ CCI_REG8(0x0c25), 0x14 },
	{ CCI_REG8(0x0c55), 0x14 },
	{ CCI_REG8(0x013e), 0x34 },
	{ CCI_REG8(0x025b), 0x00 },
	{ CCI_REG8(0x025c), 0x40 },
	{ CCI_REG8(0x02c0), 0x55 },
	{ CCI_REG8(0x02c1), 0x71 },
	{ CCI_REG8(0x02c2), 0x07 },
	{ CCI_REG8(0x02c3), 0x1c },
	{ CCI_REG8(0x0c42), 0x04 },
	{ CCI_REG8(0x0c44), 0x00 },
	{ CCI_REG8(0x0c46), 0xfd },
	{ CCI_REG8(0x0c47), 0x7f },
	{ CCI_REG8(0x0c48), 0x58 },
	{ CCI_REG8(0x0c4a), 0x18 },
	{ CCI_REG8(0x0e01), 0x42 },
	{ CCI_REG8(0x0e28), 0x5a },
	{ CCI_REG8(0x0e33), 0x00 },
	{ CCI_REG8(0x0e34), 0x04 },
	{ CCI_REG8(0x0e47), 0x02 },
	{ CCI_REG8(0x0e61), 0x20 },
	{ CCI_REG8(0x0e62), 0x20 },
	{ CCI_REG8(0x0e65), 0x01 },
	{ CCI_REG8(0x0e66), 0x20 },
	{ CCI_REG8(0x0e67), 0x2f },
	{ CCI_REG8(0x0e68), 0x2f },
	{ CCI_REG8(0x0e6a), 0x54 },
	{ CCI_REG8(0x0e6b), 0x06 },
	{ CCI_REG8(0x0e6e), 0x42 },
	{ CCI_REG8(0x0e6f), 0x44 },
	{ CCI_REG8(0x0e70), 0x2a },
	{ CCI_REG8(0x0e71), 0x2c },
	{ CCI_REG8(0x0e74), 0x45 },
	{ CCI_REG8(0x0316), 0x01 },
	{ CCI_REG8(0x0a67), 0x80 },
	{ CCI_REG8(0x00a4), 0x00 },
	{ CCI_REG8(0x00a5), 0x01 },
	{ CCI_REG8(0x00a2), 0x00 },
	{ CCI_REG8(0x00a3), 0x00 },
	{ CCI_REG8(0x00ab), 0x00 },
	{ CCI_REG8(0x00ac), 0x00 },
	{ CCI_REG8(0x00a7), 0x09 },
	{ CCI_REG8(0x00a8), 0xa0 },
	{ CCI_REG8(0x00a9), 0x0c },
	{ CCI_REG8(0x00aa), 0xd0 },
	{ CCI_REG8(0x0aaa), 0x01 },
	{ CCI_REG8(0x0aab), 0x60 },
	{ CCI_REG8(0x0aac), 0x29 },
	{ CCI_REG8(0x0aad), 0xe0 },
	{ CCI_REG8(0x0ab0), 0x0f },
	{ CCI_REG8(0x0ab1), 0x26 },
	{ CCI_REG8(0x0ab2), 0xf8 },
	{ CCI_REG8(0x0a91), 0xf2 },
	{ CCI_REG8(0x0a92), 0x12 },
	{ CCI_REG8(0x0a93), 0x64 },
	{ CCI_REG8(0x0a95), 0x41 },
	{ CCI_REG8(0x0a90), 0x17 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x02db), 0x01 },
	{ CCI_REG8(0x0b00), 0xd3 },
	{ CCI_REG8(0x0b01), 0x15 },
	{ CCI_REG8(0x0b02), 0x03 },
	{ CCI_REG8(0x0b03), 0x00 },
	{ CCI_REG8(0x0b04), 0xb4 },
	{ CCI_REG8(0x0b05), 0x0d },
	{ CCI_REG8(0x0b06), 0x0c },
	{ CCI_REG8(0x0b07), 0x01 },
	{ CCI_REG8(0x0b08), 0x02 },
	{ CCI_REG8(0x0b09), 0xa7 },
	{ CCI_REG8(0x0b0a), 0x01 },
	{ CCI_REG8(0x0b0b), 0x00 },
	{ CCI_REG8(0x0b0c), 0x11 },
	{ CCI_REG8(0x0b0d), 0x14 },
	{ CCI_REG8(0x0b0e), 0x03 },
	{ CCI_REG8(0x0b0f), 0x00 },
	{ CCI_REG8(0x0b10), 0x1b },
	{ CCI_REG8(0x0b11), 0x10 },
	{ CCI_REG8(0x0b12), 0x0c },
	{ CCI_REG8(0x0b13), 0x01 },
	{ CCI_REG8(0x0b14), 0x46 },
	{ CCI_REG8(0x0b15), 0x80 },
	{ CCI_REG8(0x0b16), 0x01 },
	{ CCI_REG8(0x0b17), 0x00 },
	{ CCI_REG8(0x0b18), 0xf0 },
	{ CCI_REG8(0x0b19), 0x81 },
	{ CCI_REG8(0x0b1a), 0x01 },
	{ CCI_REG8(0x0b1b), 0x00 },
	{ CCI_REG8(0x0b1c), 0x55 },
	{ CCI_REG8(0x0b1d), 0x84 },
	{ CCI_REG8(0x0b1e), 0x01 },
	{ CCI_REG8(0x0b1f), 0x00 },
	{ CCI_REG8(0x0b20), 0x03 },
	{ CCI_REG8(0x0b21), 0xe3 },
	{ CCI_REG8(0x0b22), 0x01 },
	{ CCI_REG8(0x0b23), 0x00 },
	{ CCI_REG8(0x0b24), 0x40 },
	{ CCI_REG8(0x0b25), 0x64 },
	{ CCI_REG8(0x0b26), 0x02 },
	{ CCI_REG8(0x0b27), 0x00 },
	{ CCI_REG8(0x0b28), 0x12 },
	{ CCI_REG8(0x0b29), 0x1c },
	{ CCI_REG8(0x0b2a), 0x03 },
	{ CCI_REG8(0x0b2b), 0x00 },
	{ CCI_REG8(0x0b2c), 0x80 },
	{ CCI_REG8(0x0b2d), 0x1c },
	{ CCI_REG8(0x0b2e), 0x03 },
	{ CCI_REG8(0x0b2f), 0x00 },
	{ CCI_REG8(0x0b30), 0x10 },
	{ CCI_REG8(0x0b31), 0xfe },
	{ CCI_REG8(0x0b32), 0x03 },
	{ CCI_REG8(0x0b33), 0x00 },
	{ CCI_REG8(0x0b34), 0x00 },
	{ CCI_REG8(0x0b35), 0xfe },
	{ CCI_REG8(0x0b36), 0x03 },
	{ CCI_REG8(0x0b37), 0x00 },
	{ CCI_REG8(0x0b38), 0x9f },
	{ CCI_REG8(0x0b39), 0x1c },
	{ CCI_REG8(0x0b3a), 0x03 },
	{ CCI_REG8(0x0b3b), 0x00 },
	{ CCI_REG8(0x0b3c), 0x00 },
	{ CCI_REG8(0x0b3d), 0xfe },
	{ CCI_REG8(0x0b3e), 0x03 },
	{ CCI_REG8(0x0b3f), 0x00 },
	{ CCI_REG8(0x0b40), 0x00 },
	{ CCI_REG8(0x0b41), 0xfe },
	{ CCI_REG8(0x0b42), 0x03 },
	{ CCI_REG8(0x0b43), 0x00 },
	{ CCI_REG8(0x0b44), 0x00 },
	{ CCI_REG8(0x0b45), 0xfe },
	{ CCI_REG8(0x0b46), 0x03 },
	{ CCI_REG8(0x0b47), 0x00 },
	{ CCI_REG8(0x0b48), 0x80 },
	{ CCI_REG8(0x0b49), 0x1c },
	{ CCI_REG8(0x0b4a), 0x03 },
	{ CCI_REG8(0x0b4b), 0x00 },
	{ CCI_REG8(0x0b4c), 0x10 },
	{ CCI_REG8(0x0b4d), 0xfe },
	{ CCI_REG8(0x0b4e), 0x03 },
	{ CCI_REG8(0x0b4f), 0x00 },
	{ CCI_REG8(0x0b50), 0x00 },
	{ CCI_REG8(0x0b51), 0xfe },
	{ CCI_REG8(0x0b52), 0x03 },
	{ CCI_REG8(0x0b53), 0x00 },
	{ CCI_REG8(0x0b54), 0x9f },
	{ CCI_REG8(0x0b55), 0x1c },
	{ CCI_REG8(0x0b56), 0x03 },
	{ CCI_REG8(0x0b57), 0x00 },
	{ CCI_REG8(0x0b58), 0x99 },
	{ CCI_REG8(0x0b59), 0x02 },
	{ CCI_REG8(0x0b5a), 0x01 },
	{ CCI_REG8(0x0b5b), 0x00 },
	{ CCI_REG8(0x0b5c), 0x00 },
	{ CCI_REG8(0x0b5d), 0x64 },
	{ CCI_REG8(0x0b5e), 0x02 },
	{ CCI_REG8(0x0b5f), 0x00 },
	{ CCI_REG8(0x0b60), 0x00 },
	{ CCI_REG8(0x0b61), 0x02 },
	{ CCI_REG8(0x0b62), 0x01 },
	{ CCI_REG8(0x0b63), 0x00 },
	{ CCI_REG8(0x0b64), 0x06 },
	{ CCI_REG8(0x0b65), 0x80 },
	{ CCI_REG8(0x0b66), 0x01 },
	{ CCI_REG8(0x0b67), 0x00 },
	{ CCI_REG8(0x0b68), 0x00 },
	{ CCI_REG8(0x0b69), 0x81 },
	{ CCI_REG8(0x0b6a), 0x01 },
	{ CCI_REG8(0x0b6b), 0x00 },
	{ CCI_REG8(0x0b6c), 0x54 },
	{ CCI_REG8(0x0b6d), 0x84 },
	{ CCI_REG8(0x0b6e), 0x01 },
	{ CCI_REG8(0x0b6f), 0x00 },
	{ CCI_REG8(0x0b70), 0x60 },
	{ CCI_REG8(0x0b71), 0x1c },
	{ CCI_REG8(0x0b72), 0x03 },
	{ CCI_REG8(0x0b73), 0x00 },
	{ CCI_REG8(0x0b74), 0x02 },
	{ CCI_REG8(0x0b75), 0xe3 },
	{ CCI_REG8(0x0b76), 0x01 },
	{ CCI_REG8(0x0b77), 0x00 },
	{ CCI_REG8(0x0b78), 0x13 },
	{ CCI_REG8(0x0b79), 0x10 },
	{ CCI_REG8(0x0b7a), 0x0c },
	{ CCI_REG8(0x0b7b), 0x01 },
	{ CCI_REG8(0x0b7c), 0x01 },
	{ CCI_REG8(0x0b7d), 0x14 },
	{ CCI_REG8(0x0b7e), 0x03 },
	{ CCI_REG8(0x0b7f), 0x00 },
	{ CCI_REG8(0x0b80), 0x00 },
	{ CCI_REG8(0x0b81), 0xa7 },
	{ CCI_REG8(0x0b82), 0x01 },
	{ CCI_REG8(0x0b83), 0x00 },
	{ CCI_REG8(0x0b84), 0x34 },
	{ CCI_REG8(0x0b85), 0x0d },
	{ CCI_REG8(0x0b86), 0x0c },
	{ CCI_REG8(0x0b87), 0x01 },
	{ CCI_REG8(0x0b88), 0x53 },
	{ CCI_REG8(0x0b89), 0x15 },
	{ CCI_REG8(0x0b8a), 0x03 },
	{ CCI_REG8(0x0b8b), 0x01 },
	{ CCI_REG8(0x0aeb), 0x09 },
	{ CCI_REG8(0x0ae9), 0x17 },
	{ CCI_REG8(0x0aea), 0x23 },
	{ CCI_REG8(0x0ae8), 0xb0 },
	{ CCI_REG8(0x05a0), 0x82 },
	{ CCI_REG8(0x05ac), 0x00 },
	{ CCI_REG8(0x05ad), 0x01 },
	{ CCI_REG8(0x0597), 0x45 },
	{ CCI_REG8(0x05ab), 0x0a },
	{ CCI_REG8(0x05a3), 0x06 },
	{ CCI_REG8(0x05a4), 0x08 },
	{ CCI_REG8(0x05ae), 0x00 },
	{ CCI_REG8(0x0800), 0x0a },
	{ CCI_REG8(0x0801), 0x14 },
	{ CCI_REG8(0x0802), 0x22 },
	{ CCI_REG8(0x0803), 0x30 },
	{ CCI_REG8(0x0804), 0x42 },
	{ CCI_REG8(0x0805), 0x0e },
	{ CCI_REG8(0x0806), 0x66 },
	{ CCI_REG8(0x0807), 0x0e },
	{ CCI_REG8(0x0808), 0x65 },
	{ CCI_REG8(0x0809), 0x02 },
	{ CCI_REG8(0x080a), 0xc3 },
	{ CCI_REG8(0x080b), 0x02 },
	{ CCI_REG8(0x080c), 0xc7 },
	{ CCI_REG8(0x080d), 0x02 },
	{ CCI_REG8(0x080e), 0xcb },
	{ CCI_REG8(0x080f), 0x0e },
	{ CCI_REG8(0x0810), 0x6c },
	{ CCI_REG8(0x0811), 0x0e },
	{ CCI_REG8(0x0812), 0x6d },
	{ CCI_REG8(0x0813), 0x00 },
	{ CCI_REG8(0x0814), 0xc0 },
	{ CCI_REG8(0x0815), 0x16 },
	{ CCI_REG8(0x0816), 0x01 },
	{ CCI_REG8(0x0817), 0x1c },
	{ CCI_REG8(0x0818), 0x1c },
	{ CCI_REG8(0x0819), 0x1c },
	{ CCI_REG8(0x081a), 0x08 },
	{ CCI_REG8(0x081b), 0x08 },
	{ CCI_REG8(0x081c), 0x00 },
	{ CCI_REG8(0x081d), 0x08 },
	{ CCI_REG8(0x081e), 0x01 },
	{ CCI_REG8(0x081f), 0x3c },
	{ CCI_REG8(0x0820), 0x3c },
	{ CCI_REG8(0x0821), 0x3c },
	{ CCI_REG8(0x0822), 0x08 },
	{ CCI_REG8(0x0823), 0x08 },
	{ CCI_REG8(0x0824), 0x00 },
	{ CCI_REG8(0x0825), 0x12 },
	{ CCI_REG8(0x0826), 0x01 },
	{ CCI_REG8(0x0827), 0x04 },
	{ CCI_REG8(0x0828), 0x04 },
	{ CCI_REG8(0x0829), 0x04 },
	{ CCI_REG8(0x082a), 0x12 },
	{ CCI_REG8(0x082b), 0x12 },
	{ CCI_REG8(0x082c), 0x00 },
	{ CCI_REG8(0x082d), 0x0e },
	{ CCI_REG8(0x082e), 0x01 },
	{ CCI_REG8(0x082f), 0x04 },
	{ CCI_REG8(0x0830), 0x04 },
	{ CCI_REG8(0x0831), 0x04 },
	{ CCI_REG8(0x0832), 0x12 },
	{ CCI_REG8(0x0833), 0x12 },
	{ CCI_REG8(0x0834), 0x00 },
	{ CCI_REG8(0x0835), 0x0b },
	{ CCI_REG8(0x0836), 0x01 },
	{ CCI_REG8(0x0837), 0x04 },
	{ CCI_REG8(0x0838), 0x04 },
	{ CCI_REG8(0x0839), 0x04 },
	{ CCI_REG8(0x083a), 0x01 },
	{ CCI_REG8(0x083b), 0x01 },
	{ CCI_REG8(0x083c), 0x02 },
	{ CCI_REG8(0x083d), 0x06 },
	{ CCI_REG8(0x083e), 0x01 },
	{ CCI_REG8(0x083f), 0x04 },
	{ CCI_REG8(0x0840), 0x04 },
	{ CCI_REG8(0x0841), 0x04 },
	{ CCI_REG8(0x0842), 0x01 },
	{ CCI_REG8(0x0843), 0x01 },
	{ CCI_REG8(0x0844), 0x02 },
	{ CCI_REG8(0x0845), 0x01 },
	{ CCI_REG8(0x0846), 0x00 },
	{ CCI_REG8(0x0847), 0x00 },
	{ CCI_REG8(0x0848), 0x00 },
	{ CCI_REG8(0x0849), 0x01 },
	{ CCI_REG8(0x084a), 0x68 },
	{ CCI_REG8(0x084b), 0x00 },
	{ CCI_REG8(0x084c), 0x01 },
	{ CCI_REG8(0x084d), 0x01 },
	{ CCI_REG8(0x084e), 0xf8 },
	{ CCI_REG8(0x084f), 0x00 },
	{ CCI_REG8(0x0850), 0x02 },
	{ CCI_REG8(0x0851), 0x02 },
	{ CCI_REG8(0x0852), 0xcc },
	{ CCI_REG8(0x0853), 0x00 },
	{ CCI_REG8(0x0854), 0x03 },
	{ CCI_REG8(0x0855), 0x03 },
	{ CCI_REG8(0x0856), 0xe8 },
	{ CCI_REG8(0x0857), 0x00 },
	{ CCI_REG8(0x0858), 0x04 },
	{ CCI_REG8(0x0859), 0x05 },
	{ CCI_REG8(0x085a), 0x98 },
	{ CCI_REG8(0x085b), 0x00 },
	{ CCI_REG8(0x085c), 0x05 },
	{ CCI_REG8(0x085d), 0x07 },
	{ CCI_REG8(0x085e), 0xc8 },
	{ CCI_REG8(0x085f), 0x00 },
	{ CCI_REG8(0x0860), 0x06 },
	{ CCI_REG8(0x0861), 0x0b },
	{ CCI_REG8(0x0862), 0x42 },
	{ CCI_REG8(0x0863), 0x10 },
	{ CCI_REG8(0x0864), 0x05 },
	{ CCI_REG8(0x0865), 0x0f },
	{ CCI_REG8(0x0866), 0xa8 },
	{ CCI_REG8(0x0867), 0x10 },
	{ CCI_REG8(0x0868), 0x06 },
	{ CCI_REG8(0x0869), 0x1e },
	{ CCI_REG8(0x086a), 0xd4 },
	{ CCI_REG8(0x086b), 0x18 },
	{ CCI_REG8(0x086c), 0x06 },
	{ CCI_REG8(0x05a0), 0xc2 },
	{ CCI_REG8(0x05ac), 0x01 },
	{ CCI_REG8(0x05ae), 0x00 },
	{ CCI_REG8(0x0207), 0x04 },
	{ CCI_REG8(0x0070), 0x05 },
	{ CCI_REG8(0x0080), 0xd0 },
	{ CCI_REG8(0x0089), 0x83 },
	{ CCI_REG8(0x009a), 0x00 },
	{ CCI_REG8(0x00a0), 0x03 },
	{ CCI_REG8(0x0c20), 0x10 },
	{ CCI_REG8(0x0c21), 0xc8 },
	{ CCI_REG8(0x0c22), 0xc8 },
	{ CCI_REG8(0x0c50), 0x10 },
	{ CCI_REG8(0x0c51), 0xc8 },
	{ CCI_REG8(0x0c52), 0xc8 },
	{ CCI_REG8(0x0040), 0x22 },
	{ CCI_REG8(0x0041), 0x20 },
	{ CCI_REG8(0x0042), 0x20 },
	{ CCI_REG8(0x0043), 0x0f },
	{ CCI_REG8(0x0044), 0x00 },
	{ CCI_REG8(0x0046), 0x0c },
	{ CCI_REG8(0x0049), 0x06 },
	{ CCI_REG8(0x004a), 0x19 },
	{ CCI_REG8(0x004d), 0x00 },
	{ CCI_REG8(0x004e), 0x03 },
	{ CCI_REG8(0x0051), 0x26 },
	{ CCI_REG8(0x005a), 0x0c },
	{ CCI_REG8(0x005b), 0x03 },
	{ CCI_REG8(0x021a), 0x00 },
	{ CCI_REG8(0x0450), 0x02 },
	{ CCI_REG8(0x0452), 0x02 },
	{ CCI_REG8(0x0454), 0x02 },
	{ CCI_REG8(0x0456), 0x02 },
	{ CCI_REG8(0x0204), 0x04 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0208), 0x01 },
	{ CCI_REG8(0x0209), 0x74 },
	{ CCI_REG8(0x0096), 0x81 },
	{ CCI_REG8(0x0097), 0x01 },
	{ CCI_REG8(0x0098), 0x87 },
	{ CCI_REG8(0x00c0), 0x00 },
	{ CCI_REG8(0x00c1), 0x80 },
	{ CCI_REG8(0x00c2), 0x11 },
	{ CCI_REG8(0x00c3), 0x00 },
	{ CCI_REG8(0x0480), 0x04 },
	{ CCI_REG8(0x0482), 0x06 },
	{ CCI_REG8(0x0484), 0x10 },
	{ CCI_REG8(0x0486), 0x10 },
	{ CCI_REG8(0x0488), 0x10 },
	{ CCI_REG8(0x048a), 0x0c },
	{ CCI_REG8(0x048c), 0x0c },
	{ CCI_REG8(0x048e), 0x0c },
	{ CCI_REG8(0x0481), 0x03 },
	{ CCI_REG8(0x0483), 0x04 },
	{ CCI_REG8(0x0485), 0x05 },
	{ CCI_REG8(0x0487), 0x05 },
	{ CCI_REG8(0x0489), 0x05 },
	{ CCI_REG8(0x048b), 0x05 },
	{ CCI_REG8(0x048d), 0x05 },
	{ CCI_REG8(0x048f), 0x05 },
	{ CCI_REG8(0x0490), 0x04 },
	{ CCI_REG8(0x0492), 0x10 },
	{ CCI_REG8(0x0494), 0x18 },
	{ CCI_REG8(0x0496), 0x28 },
	{ CCI_REG8(0x0498), 0x2c },
	{ CCI_REG8(0x049a), 0x30 },
	{ CCI_REG8(0x049c), 0x40 },
	{ CCI_REG8(0x049e), 0x40 },
	{ CCI_REG8(0x0491), 0x04 },
	{ CCI_REG8(0x0493), 0x05 },
	{ CCI_REG8(0x0495), 0x05 },
	{ CCI_REG8(0x0497), 0x05 },
	{ CCI_REG8(0x0499), 0x05 },
	{ CCI_REG8(0x049b), 0x05 },
	{ CCI_REG8(0x049d), 0x05 },
	{ CCI_REG8(0x049f), 0x05 },
	{ CCI_REG8(0x0351), 0x00 },
	{ CCI_REG8(0x0352), 0x08 },
	{ CCI_REG8(0x0353), 0x00 },
	{ CCI_REG8(0x0354), 0x08 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0x90 },
	{ CCI_REG8(0x0180), 0x46 },
	{ CCI_REG8(0x0181), 0xf0 },
	{ CCI_REG8(0x0182), 0x55 },
	{ CCI_REG8(0x0183), 0x55 },
	{ CCI_REG8(0x0184), 0x55 },
	{ CCI_REG8(0x0186), 0x5f },
	{ CCI_REG8(0x0187), 0x00 },
	{ CCI_REG8(0x0188), 0x00 },
	{ CCI_REG8(0x0189), 0x00 },
	{ CCI_REG8(0x0107), 0x00 },
	{ CCI_REG8(0x010b), 0x12 },
	{ CCI_REG8(0x0115), 0x00 },
	{ CCI_REG8(0x0121), 0x12 },
	{ CCI_REG8(0x0122), 0x07 },
	{ CCI_REG8(0x0123), 0x1f },
	{ CCI_REG8(0x0124), 0x02 },
	{ CCI_REG8(0x0125), 0x16 },
	{ CCI_REG8(0x0126), 0x08 },
	{ CCI_REG8(0x0127), 0x10 },
	{ CCI_REG8(0x0129), 0x07 },
	{ CCI_REG8(0x012a), 0x1f },
	{ CCI_REG8(0x012b), 0x08 },
	{ CCI_REG8(0x0084), 0x10 },
	{ CCI_REG8(0x0a93), 0x60 },
	{ CCI_REG8(0x0a90), 0x11 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0a90), 0x00 },
	{ CCI_REG8(0x00a4), 0x80 },
	{ CCI_REG8(0x0316), 0x00 },
	{ CCI_REG8(0x0a67), 0x00 },
	{ CCI_REG8(0x031c), 0x12 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x0102), 0x99 },
	{ CCI_REG8(0x0266), 0x52 },
};

static const s64 gc32e1_link_freq[] = {
	GC32E1_LINK_FREQ,
};

static const char * const gc32e1_test_pattern_menu[] = {
	"Disabled",
	"Colour Bars",
};

struct gc32e1 {
	struct device *dev;
	struct regmap *regmap;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc32e1_supply_names)];
};

#define to_gc32e1(_sd) container_of(_sd, struct gc32e1, sd)

static int gc32e1_power_on(struct device *dev)
{
	struct gc32e1 *sensor = to_gc32e1(dev_get_drvdata(dev));
	int ret, i;

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	fsleep(1000);

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret)
			goto err_supplies;
		fsleep(1000);
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	fsleep(1000);

	ret = clk_prepare_enable(sensor->mclk);
	if (ret)
		goto err_supplies;
	fsleep(3000);

	return 0;

err_supplies:
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	while (--i >= 0)
		regulator_disable(sensor->supplies[i].consumer);
	return ret;
}

static int gc32e1_power_off(struct device *dev)
{
	struct gc32e1 *sensor = to_gc32e1(dev_get_drvdata(dev));
	int i;

	clk_disable_unprepare(sensor->mclk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	for (i = ARRAY_SIZE(sensor->supplies) - 1; i >= 0; i--)
		regulator_disable(sensor->supplies[i].consumer);

	return 0;
}

static void gc32e1_write16(struct gc32e1 *sensor, u16 reg, u16 val,
			   int *err)
{
	cci_write(sensor->regmap, CCI_REG8(reg), val >> 8, err);
	cci_write(sensor->regmap, CCI_REG8(reg + 1), val & 0xff, err);
}

static u32 gc32e1_mbus_code(struct gc32e1 *sensor)
{
	static const u32 codes[] = {
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
	};

	return codes[sensor->vflip->val << 1 | sensor->hflip->val];
}

static int gc32e1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc32e1 *sensor = container_of(ctrl->handler, struct gc32e1,
					     ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int max = GC32E1_HEIGHT + ctrl->val - GC32E1_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       GC32E1_EXPOSURE_MIN, max, 1,
					       min(sensor->exposure->default_value,
						   (s64)max));
		if (ret)
			return ret;
	}

	if (ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP) {
		struct v4l2_subdev_state *state =
			v4l2_subdev_get_locked_active_state(&sensor->sd);

		v4l2_subdev_state_get_format(state, 0)->code =
			gc32e1_mbus_code(sensor);
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		gc32e1_write16(sensor, GC32E1_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc32e1_write16(sensor, GC32E1_REG_ANALOGUE_GAIN, ctrl->val,
			       &ret);
		break;
	case V4L2_CID_VBLANK:
		gc32e1_write16(sensor, GC32E1_REG_VTS,
			       GC32E1_HEIGHT + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, GC32E1_REG_TEST_PATTERN_IDX,
			  GC32E1_TEST_PATTERN_COLOUR_BARS, &ret);
		cci_write(sensor->regmap, GC32E1_REG_TEST_PATTERN_EN,
			  ctrl->val ? 1 : 0, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, GC32E1_REG_ORIENTATION,
			  sensor->vflip->val << 1 | sensor->hflip->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc32e1_ctrl_ops = {
	.s_ctrl = gc32e1_set_ctrl,
};

static int gc32e1_init_controls(struct gc32e1 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int vblank = GC32E1_VTS_DEF - GC32E1_HEIGHT;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, GC32E1_PIXEL_RATE,
			  GC32E1_PIXEL_RATE, 1, GC32E1_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      gc32e1_link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
				 GC32E1_HTS - GC32E1_WIDTH,
				 GC32E1_HTS - GC32E1_WIDTH, 1,
				 GC32E1_HTS - GC32E1_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   GC32E1_VTS_MAX - GC32E1_HEIGHT, 1,
					   vblank);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC32E1_EXPOSURE_MIN,
					     GC32E1_VTS_DEF - GC32E1_EXPOSURE_MARGIN,
					     1, GC32E1_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC32E1_AGAIN_MIN, GC32E1_AGAIN_MAX, 1, GC32E1_AGAIN_MIN);

	v4l2_ctrl_new_std_menu_items(hdl, &gc32e1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc32e1_test_pattern_menu) - 1,
				     0, 0, gc32e1_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops, V4L2_CID_HFLIP,
					  0, 1, 1, 0);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->vflip = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops, V4L2_CID_VFLIP,
					  0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &gc32e1_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	sensor->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int gc32e1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = gc32e1_mbus_code(to_gc32e1(sd));

	return 0;
}

static int gc32e1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != gc32e1_mbus_code(to_gc32e1(sd)))
		return -EINVAL;

	fse->min_width = GC32E1_WIDTH;
	fse->max_width = GC32E1_WIDTH;
	fse->min_height = GC32E1_HEIGHT;
	fse->max_height = GC32E1_HEIGHT;

	return 0;
}

static int gc32e1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	fmt->format = (struct v4l2_mbus_framefmt) {
		.width = GC32E1_WIDTH,
		.height = GC32E1_HEIGHT,
		.code = gc32e1_mbus_code(to_gc32e1(sd)),
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	return 0;
}

static int gc32e1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = (struct v4l2_rect) { 0, 0, 6528, 4896 };
		return 0;
	}

	return -EINVAL;
}

static int gc32e1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc32e1 *sensor = to_gc32e1(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret)
		return ret;

	cci_multi_reg_write(sensor->regmap, gc32e1_init_regs,
			    ARRAY_SIZE(gc32e1_init_regs), &ret);
	cci_multi_reg_write(sensor->regmap, gc32e1_3264x2448_regs,
			    ARRAY_SIZE(gc32e1_3264x2448_regs), &ret);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto err_put;

	ret = cci_write(sensor->regmap, GC32E1_REG_MODE_SELECT,
			GC32E1_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	__v4l2_ctrl_grab(sensor->hflip, true);
	__v4l2_ctrl_grab(sensor->vflip, true);
	fsleep(5000);

	return 0;

err_put:
	pm_runtime_put_autosuspend(sensor->dev);
	return ret;
}

static int gc32e1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc32e1 *sensor = to_gc32e1(sd);
	int ret;

	ret = cci_write(sensor->regmap, GC32E1_REG_MODE_SELECT,
			GC32E1_MODE_STANDBY, NULL);
	__v4l2_ctrl_grab(sensor->hflip, false);
	__v4l2_ctrl_grab(sensor->vflip, false);
	pm_runtime_put_autosuspend(sensor->dev);

	return ret;
}

static int gc32e1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_TRY };

	return gc32e1_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops gc32e1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc32e1_pad_ops = {
	.enum_mbus_code = gc32e1_enum_mbus_code,
	.enum_frame_size = gc32e1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc32e1_set_fmt,
	.get_selection = gc32e1_get_selection,
	.enable_streams = gc32e1_enable_streams,
	.disable_streams = gc32e1_disable_streams,
};

static const struct v4l2_subdev_ops gc32e1_subdev_ops = {
	.video = &gc32e1_video_ops,
	.pad = &gc32e1_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc32e1_internal_ops = {
	.init_state = gc32e1_init_state,
};

static int gc32e1_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "cannot parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC32E1_LANES)
		ret = dev_err_probe(dev, -EINVAL, "need %u data lanes\n",
				    GC32E1_LANES);
	else
		ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
					       bus_cfg.nr_of_link_frequencies,
					       gc32e1_link_freq,
					       ARRAY_SIZE(gc32e1_link_freq),
					       &(unsigned long){ 0 });

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc32e1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc32e1 *sensor;
	u64 id_h, id_l, id = 0;
	int ret, i;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->dev = dev;

	ret = gc32e1_check_endpoint(dev);
	if (ret)
		return ret;

	sensor->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "cannot get clock\n");
	if (clk_get_rate(sensor->mclk) != GC32E1_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "clock must be 24 MHz\n");

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = gc32e1_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get supplies\n");

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	v4l2_i2c_subdev_init(&sensor->sd, client, &gc32e1_subdev_ops);

	ret = gc32e1_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(sensor->regmap, GC32E1_REG_CHIP_ID_H, &id_h, NULL);
	cci_read(sensor->regmap, GC32E1_REG_CHIP_ID_L, &id_l, &ret);
	if (!ret)
		id = id_h << 8 | id_l;
	if (!ret && id != GC32E1_CHIP_ID)
		ret = -ENODEV;
	if (ret) {
		dev_err_probe(dev, ret, "chip ID 0x%llx, expected 0x%x\n", id,
			      GC32E1_CHIP_ID);
		goto err_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = gc32e1_init_controls(sensor);
	if (ret)
		goto err_pm;

	sensor->sd.internal_ops = &gc32e1_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrls;

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_entity;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_subdev;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&sensor->sd);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_power_off:
	gc32e1_power_off(dev);
	return ret;
}

static void gc32e1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc32e1 *sensor = to_gc32e1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev)) {
		gc32e1_power_off(sensor->dev);
		pm_runtime_set_suspended(sensor->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc32e1_pm_ops, gc32e1_power_off,
				 gc32e1_power_on, NULL);

static const struct of_device_id gc32e1_of_match[] = {
	{ .compatible = "galaxycore,gc32e1" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc32e1_of_match);

static struct i2c_driver gc32e1_driver = {
	.driver = {
		.name = "gc32e1",
		.of_match_table = gc32e1_of_match,
		.pm = pm_ptr(&gc32e1_pm_ops),
	},
	.probe = gc32e1_probe,
	.remove = gc32e1_remove,
};
module_i2c_driver(gc32e1_driver);

MODULE_DESCRIPTION("GalaxyCore GC32E1 sensor driver");
MODULE_AUTHOR("Marcus Ramberg <marcus@means.no>");
MODULE_LICENSE("GPL");
