// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX882 50 MP RAW sensor driver
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

#define IMX882_LANES			4
#define IMX882_MCLK_FREQ		24000000
#define IMX882_VTS_MAX			0xffff

#define IMX882_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX882_CHIP_ID			0x8202

/* bit 0 mirror, bit 1 flip; the Bayer order follows */
#define IMX882_REG_ORIENTATION		CCI_REG8(0x0101)

#define IMX882_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX882_MODE_STANDBY		0x00
#define IMX882_MODE_STREAMING		0x01

#define IMX882_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define IMX882_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX882_EXPOSURE_MIN		8
#define IMX882_EXPOSURE_MARGIN		48
#define IMX882_EXPOSURE_DEF		1000

/* gain = 16384 / (16384 - code) */
#define IMX882_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
#define IMX882_AGAIN_MAX		15360

#define IMX882_REG_VTS			CCI_REG16(0x0340)
#define IMX882_REG_TEST_PATTERN		CCI_REG16(0x0600)

static const char * const imx882_supply_names[] = {
	/* power-up order */
	"dovdd",
	"dvdd",
	"avdd",
};

static const struct cci_reg_sequence imx882_init_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x961c), 0x04 },
	{ CCI_REG8(0xf800), 0x18 },
	{ CCI_REG8(0xf801), 0x18 },
	{ CCI_REG8(0xf802), 0xc5 },
	{ CCI_REG8(0xf803), 0x26 },
	{ CCI_REG8(0xf804), 0x55 },
	{ CCI_REG8(0xf805), 0xc4 },
	{ CCI_REG8(0xf806), 0x3f },
	{ CCI_REG8(0xf807), 0x00 },
	{ CCI_REG8(0xf808), 0x17 },
	{ CCI_REG8(0xf809), 0xfa },
	{ CCI_REG8(0xf80a), 0x84 },
	{ CCI_REG8(0xf80b), 0xbe },
	{ CCI_REG8(0xf80c), 0x55 },
	{ CCI_REG8(0xf80d), 0xc8 },
	{ CCI_REG8(0xf80e), 0x3f },
	{ CCI_REG8(0xf80f), 0x00 },
	{ CCI_REG8(0xf810), 0x40 },
	{ CCI_REG8(0xf811), 0x80 },
	{ CCI_REG8(0xf812), 0x3a },
	{ CCI_REG8(0xf813), 0xc4 },
	{ CCI_REG8(0xfa00), 0x23 },
	{ CCI_REG8(0xfa01), 0x23 },
	{ CCI_REG8(0xfa02), 0xc5 },
	{ CCI_REG8(0xfa03), 0x2b },
	{ CCI_REG8(0xfa04), 0x55 },
	{ CCI_REG8(0xfa05), 0xc4 },
	{ CCI_REG8(0xfa06), 0x3f },
	{ CCI_REG8(0xfa07), 0x00 },
	{ CCI_REG8(0xfa08), 0x17 },
	{ CCI_REG8(0xfa09), 0xfa },
	{ CCI_REG8(0xfa0a), 0x84 },
	{ CCI_REG8(0xfa0b), 0xbe },
	{ CCI_REG8(0xfa0c), 0x55 },
	{ CCI_REG8(0xfa0d), 0xc8 },
	{ CCI_REG8(0xfa0e), 0x3f },
	{ CCI_REG8(0xfa0f), 0x00 },
	{ CCI_REG8(0xfa10), 0x40 },
	{ CCI_REG8(0xfa11), 0x80 },
	{ CCI_REG8(0xfa12), 0x3a },
	{ CCI_REG8(0xfa13), 0xc4 },
	{ CCI_REG8(0xfc00), 0x4f },
	{ CCI_REG8(0xfc01), 0x06 },
	{ CCI_REG8(0xfc02), 0xac },
	{ CCI_REG8(0xfc03), 0x27 },
	{ CCI_REG8(0xfc04), 0xac },
	{ CCI_REG8(0xfc05), 0x1d },
	{ CCI_REG8(0xfc06), 0xac },
	{ CCI_REG8(0xfc07), 0x14 },
	{ CCI_REG8(0xfc08), 0xfa },
	{ CCI_REG8(0xfc09), 0xe8 },
	{ CCI_REG8(0xfc0a), 0xeb },
	{ CCI_REG8(0xfc0b), 0x2e },
	{ CCI_REG8(0xfc0c), 0xd0 },
	{ CCI_REG8(0xfc0d), 0x20 },
	{ CCI_REG8(0xfc0e), 0x40 },
	{ CCI_REG8(0xfc0f), 0x4f },
	{ CCI_REG8(0xfc10), 0x1e },
	{ CCI_REG8(0xfc11), 0x05 },
	{ CCI_REG8(0xfc12), 0xd4 },
	{ CCI_REG8(0xfc13), 0x18 },
	{ CCI_REG8(0xfc14), 0x08 },
	{ CCI_REG8(0xfc15), 0x00 },
	{ CCI_REG8(0xfc16), 0xf6 },
	{ CCI_REG8(0xfc17), 0x40 },
	{ CCI_REG8(0xfc18), 0x06 },
	{ CCI_REG8(0xfc19), 0xb7 },
	{ CCI_REG8(0xfc1a), 0xbe },
	{ CCI_REG8(0xfc1b), 0x06 },
	{ CCI_REG8(0xfc1c), 0xf0 },
	{ CCI_REG8(0xfc1d), 0x40 },
	{ CCI_REG8(0xfc1e), 0x06 },
	{ CCI_REG8(0xfc1f), 0xb8 },
	{ CCI_REG8(0xfc20), 0x29 },
	{ CCI_REG8(0xfc21), 0x25 },
	{ CCI_REG8(0xfc22), 0x90 },
	{ CCI_REG8(0xfc23), 0x2c },
	{ CCI_REG8(0xfc24), 0x80 },
	{ CCI_REG8(0xfc25), 0x22 },
	{ CCI_REG8(0xfc26), 0xf6 },
	{ CCI_REG8(0xfc27), 0x41 },
	{ CCI_REG8(0xfc28), 0x05 },
	{ CCI_REG8(0xfc29), 0x1c },
	{ CCI_REG8(0xfc2a), 0xb8 },
	{ CCI_REG8(0xfc2b), 0x8b },
	{ CCI_REG8(0xfc2c), 0xd0 },
	{ CCI_REG8(0xfc2d), 0x18 },
	{ CCI_REG8(0xfc2e), 0x27 },
	{ CCI_REG8(0xfc2f), 0x6c },
	{ CCI_REG8(0xfc30), 0xf0 },
	{ CCI_REG8(0xfc31), 0x02 },
	{ CCI_REG8(0xfc32), 0x02 },
	{ CCI_REG8(0xfc33), 0x74 },
	{ CCI_REG8(0xfc34), 0xf6 },
	{ CCI_REG8(0xfc35), 0x40 },
	{ CCI_REG8(0xfc36), 0x07 },
	{ CCI_REG8(0xfc37), 0x10 },
	{ CCI_REG8(0xfc38), 0x0f },
	{ CCI_REG8(0xfc39), 0x20 },
	{ CCI_REG8(0xfc3a), 0xfa },
	{ CCI_REG8(0xfc3b), 0xe8 },
	{ CCI_REG8(0xfc3c), 0x03 },
	{ CCI_REG8(0xfc3d), 0x8a },
	{ CCI_REG8(0xfc3e), 0x80 },
	{ CCI_REG8(0xfc3f), 0x04 },
	{ CCI_REG8(0xfc40), 0x5f },
	{ CCI_REG8(0xfc41), 0xf0 },
	{ CCI_REG8(0xfc42), 0x29 },
	{ CCI_REG8(0xfc43), 0x25 },
	{ CCI_REG8(0xfc44), 0x90 },
	{ CCI_REG8(0xfc45), 0x0a },
	{ CCI_REG8(0xfc46), 0x50 },
	{ CCI_REG8(0xfc47), 0x31 },
	{ CCI_REG8(0xfc48), 0xfa },
	{ CCI_REG8(0xfc49), 0xe8 },
	{ CCI_REG8(0xfc4a), 0x03 },
	{ CCI_REG8(0xfc4b), 0x7c },
	{ CCI_REG8(0xfc4c), 0x80 },
	{ CCI_REG8(0xfc4d), 0x04 },
	{ CCI_REG8(0xfc4e), 0x09 },
	{ CCI_REG8(0xfc4f), 0x50 },
	{ CCI_REG8(0xfc50), 0xf4 },
	{ CCI_REG8(0xfc51), 0x40 },
	{ CCI_REG8(0xfc52), 0x06 },
	{ CCI_REG8(0xfc53), 0xbc },
	{ CCI_REG8(0xfc54), 0x50 },
	{ CCI_REG8(0xfc55), 0xa1 },
	{ CCI_REG8(0xfc56), 0xfa },
	{ CCI_REG8(0xfc57), 0xe8 },
	{ CCI_REG8(0xfc58), 0x03 },
	{ CCI_REG8(0xfc59), 0x6e },
	{ CCI_REG8(0xfc5a), 0xfa },
	{ CCI_REG8(0xfc5b), 0xe8 },
	{ CCI_REG8(0xfc5c), 0x02 },
	{ CCI_REG8(0xfc5d), 0x9e },
	{ CCI_REG8(0xfc5e), 0xa8 },
	{ CCI_REG8(0xfc5f), 0x14 },
	{ CCI_REG8(0xfc60), 0xa8 },
	{ CCI_REG8(0xfc61), 0x1d },
	{ CCI_REG8(0xfc62), 0xa8 },
	{ CCI_REG8(0xfc63), 0x27 },
	{ CCI_REG8(0xfc64), 0xa0 },
	{ CCI_REG8(0xfc65), 0x09 },
	{ CCI_REG8(0xfc66), 0x00 },
	{ CCI_REG8(0xfc67), 0x00 },
	{ CCI_REG8(0x4331), 0x01 },
	{ CCI_REG8(0x3304), 0x00 },
	{ CCI_REG8(0x33f0), 0x01 },
	{ CCI_REG8(0x33f1), 0x07 },
	{ CCI_REG8(0x0111), 0x02 },
	{ CCI_REG8(0x3855), 0x01 },
	{ CCI_REG8(0x0d06), 0x82 },
	{ CCI_REG8(0x0d07), 0x02 },
	{ CCI_REG8(0x0d12), 0x78 },
	{ CCI_REG8(0x0d13), 0x01 },
	{ CCI_REG8(0x0d0d), 0x00 },
	{ CCI_REG8(0x0d08), 0x01 },
	{ CCI_REG8(0x3b32), 0x12 },
	{ CCI_REG8(0x3b40), 0x00 },
	{ CCI_REG8(0x7816), 0x01 },
	{ CCI_REG8(0x7819), 0x01 },
	{ CCI_REG8(0x781c), 0x01 },
	{ CCI_REG8(0x781d), 0x01 },
	{ CCI_REG8(0x781e), 0x01 },
	{ CCI_REG8(0x7821), 0x01 },
	{ CCI_REG8(0x7824), 0x01 },
	{ CCI_REG8(0x7827), 0x01 },
	{ CCI_REG8(0x7828), 0x01 },
	{ CCI_REG8(0x7829), 0x01 },
	{ CCI_REG8(0x782c), 0x0a },
	{ CCI_REG8(0x782f), 0x0a },
	{ CCI_REG8(0x7832), 0x0a },
	{ CCI_REG8(0x7833), 0x0a },
	{ CCI_REG8(0x7834), 0x0a },
	{ CCI_REG8(0x7837), 0x19 },
	{ CCI_REG8(0x783a), 0x19 },
	{ CCI_REG8(0x783d), 0x19 },
	{ CCI_REG8(0x783e), 0x19 },
	{ CCI_REG8(0x783f), 0x19 },
	{ CCI_REG8(0x7841), 0x1d },
	{ CCI_REG8(0x7843), 0x1d },
	{ CCI_REG8(0x7846), 0x1d },
	{ CCI_REG8(0x7847), 0x1d },
	{ CCI_REG8(0x7848), 0x1d },
	{ CCI_REG8(0xe24e), 0x00 },
	{ CCI_REG8(0x7854), 0x50 },
	{ CCI_REG8(0x7859), 0x50 },
	{ CCI_REG8(0x785e), 0x50 },
	{ CCI_REG8(0x7863), 0x50 },
	{ CCI_REG8(0x7869), 0x50 },
	{ CCI_REG8(0x7a5e), 0x00 },
	{ CCI_REG8(0x7a5f), 0x32 },
	{ CCI_REG8(0x7a66), 0x00 },
	{ CCI_REG8(0x7a67), 0x32 },
	{ CCI_REG8(0x7a6c), 0x00 },
	{ CCI_REG8(0x7a6d), 0x3e },
	{ CCI_REG8(0x7a74), 0x00 },
	{ CCI_REG8(0x7a75), 0x3e },
	{ CCI_REG8(0x7aae), 0x00 },
	{ CCI_REG8(0x7aaf), 0x99 },
	{ CCI_REG8(0x7ab6), 0x00 },
	{ CCI_REG8(0x7ab7), 0x99 },
	{ CCI_REG8(0x7abc), 0x00 },
	{ CCI_REG8(0x7abd), 0xa5 },
	{ CCI_REG8(0x7ac4), 0x00 },
	{ CCI_REG8(0x7ac5), 0xa5 },
	{ CCI_REG8(0x7b00), 0x00 },
	{ CCI_REG8(0x7b01), 0x66 },
	{ CCI_REG8(0x7b08), 0x00 },
	{ CCI_REG8(0x7b09), 0x72 },
	{ CCI_REG8(0x7ca6), 0x01 },
	{ CCI_REG8(0x7ca9), 0x01 },
	{ CCI_REG8(0x7cac), 0x01 },
	{ CCI_REG8(0x7cad), 0x01 },
	{ CCI_REG8(0x7cae), 0x01 },
	{ CCI_REG8(0x7cb1), 0x01 },
	{ CCI_REG8(0x7cb4), 0x01 },
	{ CCI_REG8(0x7cb7), 0x01 },
	{ CCI_REG8(0x7cb8), 0x01 },
	{ CCI_REG8(0x7cb9), 0x01 },
	{ CCI_REG8(0x7cbc), 0x01 },
	{ CCI_REG8(0x7cbf), 0x01 },
	{ CCI_REG8(0x7cc2), 0x01 },
	{ CCI_REG8(0x7cc3), 0x01 },
	{ CCI_REG8(0x7cc4), 0x01 },
	{ CCI_REG8(0x7cd2), 0x01 },
	{ CCI_REG8(0x7cd5), 0x01 },
	{ CCI_REG8(0x7cd8), 0x01 },
	{ CCI_REG8(0x7cd9), 0x01 },
	{ CCI_REG8(0x7cda), 0x01 },
	{ CCI_REG8(0x7cdd), 0x01 },
	{ CCI_REG8(0x7ce0), 0x01 },
	{ CCI_REG8(0x7ce3), 0x01 },
	{ CCI_REG8(0x7ce4), 0x01 },
	{ CCI_REG8(0x7ce5), 0x01 },
	{ CCI_REG8(0x7ce8), 0x01 },
	{ CCI_REG8(0x7ceb), 0x01 },
	{ CCI_REG8(0x7cee), 0x01 },
	{ CCI_REG8(0x7cef), 0x01 },
	{ CCI_REG8(0x7cf0), 0x01 },
	{ CCI_REG8(0xab30), 0x00 },
	{ CCI_REG8(0xab31), 0x3c },
	{ CCI_REG8(0xab32), 0x00 },
	{ CCI_REG8(0xab33), 0x3c },
	{ CCI_REG8(0xab34), 0x00 },
	{ CCI_REG8(0xab35), 0x3c },
	{ CCI_REG8(0x0d08), 0x00 },
	{ CCI_REG8(0x0106), 0x01 },
};

static const struct cci_reg_sequence imx882_2048x1536_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0342), 0x09 },
	{ CCI_REG8(0x0343), 0xa4 },
	{ CCI_REG8(0x3850), 0x00 },
	{ CCI_REG8(0x3851), 0x8f },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x44 },
	{ CCI_REG8(0x0902), 0x02 },
	{ CCI_REG8(0x3005), 0x00 },
	{ CCI_REG8(0x3006), 0x00 },
	{ CCI_REG8(0x3140), 0x0a },
	{ CCI_REG8(0x3144), 0x00 },
	{ CCI_REG8(0x3148), 0x00 },
	{ CCI_REG8(0x31c0), 0x43 },
	{ CCI_REG8(0x31c1), 0x43 },
	{ CCI_REG8(0x3205), 0x00 },
	{ CCI_REG8(0x323c), 0x01 },
	{ CCI_REG8(0x39ac), 0x01 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x08 },
	{ CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x06 },
	{ CCI_REG8(0x040f), 0x00 },
	{ CCI_REG8(0x034c), 0x08 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x06 },
	{ CCI_REG8(0x034f), 0x00 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x04 },
	{ CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x04 },
	{ CCI_REG8(0x030b), 0x02 },
	{ CCI_REG8(0x030d), 0x0c },
	{ CCI_REG8(0x030e), 0x03 },
	{ CCI_REG8(0x030f), 0xe8 },
	{ CCI_REG8(0x3104), 0x00 },
	{ CCI_REG8(0x38a0), 0x00 },
	{ CCI_REG8(0x38a1), 0x00 },
	{ CCI_REG8(0x38a2), 0x00 },
	{ CCI_REG8(0x38a3), 0x00 },
	{ CCI_REG8(0x38a8), 0x00 },
	{ CCI_REG8(0x38a9), 0x00 },
	{ CCI_REG8(0x38aa), 0x00 },
	{ CCI_REG8(0x38ab), 0x00 },
	{ CCI_REG8(0x38b0), 0x03 },
	{ CCI_REG8(0x38b1), 0xff },
	{ CCI_REG8(0x38b4), 0x03 },
	{ CCI_REG8(0x38b5), 0xff },
	{ CCI_REG8(0x38b8), 0x03 },
	{ CCI_REG8(0x38b9), 0xff },
	{ CCI_REG8(0x38bc), 0x03 },
	{ CCI_REG8(0x38bd), 0xff },
	{ CCI_REG8(0x38d0), 0x01 },
	{ CCI_REG8(0x38d1), 0x90 },
	{ CCI_REG8(0x38d2), 0x01 },
	{ CCI_REG8(0x38d3), 0x90 },
	{ CCI_REG8(0x38d8), 0x14 },
	{ CCI_REG8(0x38e0), 0x00 },
	{ CCI_REG8(0x38e1), 0x00 },
	{ CCI_REG8(0x38e2), 0x00 },
	{ CCI_REG8(0x38e3), 0x00 },
	{ CCI_REG8(0x38e4), 0x00 },
	{ CCI_REG8(0x38e5), 0x00 },
	{ CCI_REG8(0x38e6), 0x00 },
	{ CCI_REG8(0x38e7), 0x00 },
	{ CCI_REG8(0x3b00), 0x00 },
	{ CCI_REG8(0x3b01), 0x00 },
	{ CCI_REG8(0x3b04), 0x00 },
	{ CCI_REG8(0x3b05), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x3310), 0x00 },
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3422), 0x01 },
	{ CCI_REG8(0x3423), 0xfc },
	{ CCI_REG8(0x30a2), 0x00 },
	{ CCI_REG8(0x30f1), 0x01 },
	{ CCI_REG8(0x30a3), 0x30 },
};

static const struct cci_reg_sequence imx882_4096x3072_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0342), 0x17 },
	{ CCI_REG8(0x0343), 0x64 },
	{ CCI_REG8(0x3850), 0x00 },
	{ CCI_REG8(0x3851), 0xa8 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0xff },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0xff },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x00 },
	/* 0x3005/0x3006/0x3103 as slim: stock adds PDAF lines MUX counts as image */
	{ CCI_REG8(0x3005), 0x00 },
	{ CCI_REG8(0x3006), 0x00 },
	{ CCI_REG8(0x3140), 0x0a },
	{ CCI_REG8(0x3144), 0x00 },
	{ CCI_REG8(0x3148), 0x00 },
	{ CCI_REG8(0x31c0), 0x41 },
	{ CCI_REG8(0x31c1), 0x41 },
	{ CCI_REG8(0x3205), 0x00 },
	{ CCI_REG8(0x323c), 0x02 },
	{ CCI_REG8(0x39ac), 0x01 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x10 },
	{ CCI_REG8(0x040d), 0x00 },
	{ CCI_REG8(0x040e), 0x0c },
	{ CCI_REG8(0x040f), 0x00 },
	{ CCI_REG8(0x034c), 0x10 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x0c },
	{ CCI_REG8(0x034f), 0x00 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x03 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x0c },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x0c },
	{ CCI_REG8(0x030e), 0x03 },
	{ CCI_REG8(0x030f), 0x6e },
	{ CCI_REG8(0x3104), 0x00 },
	{ CCI_REG8(0x38a0), 0x00 },
	{ CCI_REG8(0x38a1), 0x00 },
	{ CCI_REG8(0x38a2), 0x00 },
	{ CCI_REG8(0x38a3), 0x00 },
	{ CCI_REG8(0x38a8), 0x00 },
	{ CCI_REG8(0x38a9), 0x00 },
	{ CCI_REG8(0x38aa), 0x00 },
	{ CCI_REG8(0x38ab), 0x00 },
	{ CCI_REG8(0x38b0), 0x03 },
	{ CCI_REG8(0x38b1), 0xff },
	{ CCI_REG8(0x38b4), 0x03 },
	{ CCI_REG8(0x38b5), 0xff },
	{ CCI_REG8(0x38b8), 0x03 },
	{ CCI_REG8(0x38b9), 0xff },
	{ CCI_REG8(0x38bc), 0x03 },
	{ CCI_REG8(0x38bd), 0xff },
	{ CCI_REG8(0x38d0), 0x01 },
	{ CCI_REG8(0x38d1), 0xc2 },
	{ CCI_REG8(0x38d2), 0x01 },
	{ CCI_REG8(0x38d3), 0xc2 },
	{ CCI_REG8(0x38d8), 0x14 },
	{ CCI_REG8(0x38e0), 0x00 },
	{ CCI_REG8(0x38e1), 0x00 },
	{ CCI_REG8(0x38e2), 0x00 },
	{ CCI_REG8(0x38e3), 0x00 },
	{ CCI_REG8(0x38e4), 0x00 },
	{ CCI_REG8(0x38e5), 0x00 },
	{ CCI_REG8(0x38e6), 0x00 },
	{ CCI_REG8(0x38e7), 0x00 },
	{ CCI_REG8(0x3b00), 0x00 },
	{ CCI_REG8(0x3b01), 0x00 },
	{ CCI_REG8(0x3b04), 0x00 },
	{ CCI_REG8(0x3b05), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x3310), 0x00 },
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3422), 0x01 },
	{ CCI_REG8(0x3423), 0xfc },
	{ CCI_REG8(0x30a2), 0x00 },
	{ CCI_REG8(0x30f1), 0x01 },
	{ CCI_REG8(0x30a3), 0x30 },
};

/* 24 MHz / 12 * IOP_MPY / IOP_SYCK_DIV / 2 */
static const s64 imx882_link_freq[] = {
	500000000,
	878000000,
};

struct imx882_mode {
	u32 width;
	u32 height;
	/* a longer line truncates the frame instead of stretching it */
	u32 hts;
	u32 vts_min;
	u32 vts_def;
	/* from the frame interval */
	u64 pixel_rate;
	unsigned int link_freq_index;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct imx882_mode imx882_modes[] = {
	{
		/* 2x2 binned, stock capture */
		.width = 4096,
		.height = 3072,
		.hts = 5988,
		.vts_min = 4761,
		.vts_def = 4761,
		.pixel_rate = 857600000,
		.link_freq_index = 1,
		.regs = imx882_4096x3072_regs,
		.num_regs = ARRAY_SIZE(imx882_4096x3072_regs),
	}, {
		/* 4x4 binned, stock slim video; readout limited below VTS 5594 */
		.width = 2048,
		.height = 1536,
		.hts = 2468,
		.vts_min = 5594,
		.vts_def = 11188,
		.pixel_rate = 832000000,
		.link_freq_index = 0,
		.regs = imx882_2048x1536_regs,
		.num_regs = ARRAY_SIZE(imx882_2048x1536_regs),
	},
};

static const char * const imx882_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

struct imx882 {
	struct device *dev;
	struct regmap *regmap;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	const struct imx882_mode *mode;
	unsigned long link_freq_bitmap;

	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx882_supply_names)];
};

#define to_imx882(_sd) container_of(_sd, struct imx882, sd)

static int imx882_power_on(struct device *dev)
{
	struct imx882 *sensor = to_imx882(dev_get_drvdata(dev));
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
	fsleep(8000);

	return 0;

err_supplies:
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	while (--i >= 0)
		regulator_disable(sensor->supplies[i].consumer);
	return ret;
}

static int imx882_power_off(struct device *dev)
{
	struct imx882 *sensor = to_imx882(dev_get_drvdata(dev));
	int i;

	clk_disable_unprepare(sensor->mclk);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	for (i = ARRAY_SIZE(sensor->supplies) - 1; i >= 0; i--)
		regulator_disable(sensor->supplies[i].consumer);

	return 0;
}

static u32 imx882_mbus_code(struct imx882 *sensor)
{
	static const u32 codes[] = {
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
	};

	return codes[sensor->vflip->val << 1 | sensor->hflip->val];
}

static int imx882_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx882 *sensor = container_of(ctrl->handler, struct imx882,
					     ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int max = sensor->mode->height + ctrl->val -
			  IMX882_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       IMX882_EXPOSURE_MIN, max, 1,
					       min(sensor->exposure->default_value,
						   (s64)max));
		if (ret)
			return ret;
	}

	if (ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP) {
		struct v4l2_subdev_state *state =
			v4l2_subdev_get_locked_active_state(&sensor->sd);

		v4l2_subdev_state_get_format(state, 0)->code =
			imx882_mbus_code(sensor);
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, IMX882_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, IMX882_REG_ANALOGUE_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, IMX882_REG_VTS,
			  sensor->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, IMX882_REG_TEST_PATTERN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, IMX882_REG_ORIENTATION,
			  sensor->vflip->val << 1 | sensor->hflip->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx882_ctrl_ops = {
	.s_ctrl = imx882_set_ctrl,
};

static int imx882_init_controls(struct imx882 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	const struct imx882_mode *mode = sensor->mode;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					       1, INT_MAX, 1, mode->pixel_rate);

	sensor->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx882_link_freq) - 1,
						   mode->link_freq_index,
						   imx882_link_freq);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
					   mode->hts - mode->width,
					   mode->hts - mode->width, 1,
					   mode->hts - mode->width);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(hdl, &imx882_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vts_min - mode->height,
					   IMX882_VTS_MAX - mode->height, 1,
					   mode->vts_def - mode->height);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &imx882_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX882_EXPOSURE_MIN,
					     mode->vts_def - IMX882_EXPOSURE_MARGIN,
					     1, IMX882_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, &imx882_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  0, IMX882_AGAIN_MAX, 1, 0);

	v4l2_ctrl_new_std_menu_items(hdl, &imx882_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx882_test_pattern_menu) - 1,
				     0, 0, imx882_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(hdl, &imx882_ctrl_ops, V4L2_CID_HFLIP,
					  0, 1, 1, 0);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->vflip = v4l2_ctrl_new_std(hdl, &imx882_ctrl_ops, V4L2_CID_VFLIP,
					  0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &imx882_ctrl_ops, &props);

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

static int imx882_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = imx882_mbus_code(to_imx882(sd));

	return 0;
}

static bool imx882_mode_usable(struct imx882 *sensor,
			       const struct imx882_mode *mode)
{
	return test_bit(mode->link_freq_index, &sensor->link_freq_bitmap);
}

static int imx882_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx882 *sensor = to_imx882(sd);
	unsigned int i, n = 0;

	if (fse->code != imx882_mbus_code(sensor))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(imx882_modes); i++) {
		const struct imx882_mode *mode = &imx882_modes[i];

		if (!imx882_mode_usable(sensor, mode) || n++ != fse->index)
			continue;

		fse->min_width = mode->width;
		fse->max_width = mode->width;
		fse->min_height = mode->height;
		fse->max_height = mode->height;
		return 0;
	}

	return -EINVAL;
}

static const struct imx882_mode *
imx882_find_mode(struct imx882 *sensor, u32 width, u32 height)
{
	const struct imx882_mode *best = NULL;
	u32 best_dist = U32_MAX;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx882_modes); i++) {
		const struct imx882_mode *mode = &imx882_modes[i];
		u32 dist = abs((int)mode->width - (int)width) +
			   abs((int)mode->height - (int)height);

		if (imx882_mode_usable(sensor, mode) && dist < best_dist) {
			best = mode;
			best_dist = dist;
		}
	}

	return best;
}

static int imx882_update_controls(struct imx882 *sensor,
				  const struct imx882_mode *mode)
{
	int exp_max = mode->vts_def - IMX882_EXPOSURE_MARGIN;
	int hblank = mode->hts - mode->width;
	int ret;

	ret = __v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate, mode->pixel_rate);
	if (!ret)
		ret = __v4l2_ctrl_s_ctrl(sensor->link_freq,
					 mode->link_freq_index);
	if (!ret)
		ret = __v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank,
					       1, hblank);
	if (!ret)
		ret = __v4l2_ctrl_modify_range(sensor->vblank,
					       mode->vts_min - mode->height,
					       IMX882_VTS_MAX - mode->height,
					       1, mode->vts_def - mode->height);
	if (!ret)
		ret = __v4l2_ctrl_s_ctrl(sensor->vblank,
					 mode->vts_def - mode->height);
	/* s_ctrl does not run if the vblank value did not change */
	if (!ret)
		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       IMX882_EXPOSURE_MIN, exp_max, 1,
					       min(sensor->exposure->default_value,
						   (s64)exp_max));

	return ret;
}

static int imx882_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx882 *sensor = to_imx882(sd);
	const struct imx882_mode *mode;
	int ret;

	mode = imx882_find_mode(sensor, fmt->format.width,
				fmt->format.height);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && mode != sensor->mode) {
		const struct imx882_mode *old = sensor->mode;

		if (v4l2_subdev_is_streaming(sd))
			return -EBUSY;

		/* the vblank handler derives the exposure range from it */
		sensor->mode = mode;
		ret = imx882_update_controls(sensor, mode);
		if (ret) {
			sensor->mode = old;
			return ret;
		}
	}

	fmt->format = (struct v4l2_mbus_framefmt) {
		.width = mode->width,
		.height = mode->height,
		.code = imx882_mbus_code(sensor),
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	return 0;
}

static int imx882_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = (struct v4l2_rect) { 0, 0, 8192, 6144 };
		return 0;
	}

	return -EINVAL;
}

static int imx882_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx882 *sensor = to_imx882(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret)
		return ret;

	cci_multi_reg_write(sensor->regmap, imx882_init_regs,
			    ARRAY_SIZE(imx882_init_regs), &ret);
	cci_multi_reg_write(sensor->regmap, sensor->mode->regs,
			    sensor->mode->num_regs, &ret);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto err_put;

	ret = cci_write(sensor->regmap, IMX882_REG_MODE_SELECT,
			IMX882_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	__v4l2_ctrl_grab(sensor->hflip, true);
	__v4l2_ctrl_grab(sensor->vflip, true);

	return 0;

err_put:
	pm_runtime_put_autosuspend(sensor->dev);
	return ret;
}

static int imx882_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx882 *sensor = to_imx882(sd);
	int ret;

	ret = cci_write(sensor->regmap, IMX882_REG_MODE_SELECT,
			IMX882_MODE_STANDBY, NULL);
	__v4l2_ctrl_grab(sensor->hflip, false);
	__v4l2_ctrl_grab(sensor->vflip, false);
	pm_runtime_put_autosuspend(sensor->dev);

	return ret;
}

static int imx882_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct imx882 *sensor = to_imx882(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format.width = sensor->mode->width,
		.format.height = sensor->mode->height,
	};

	return imx882_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx882_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx882_pad_ops = {
	.enum_mbus_code = imx882_enum_mbus_code,
	.enum_frame_size = imx882_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx882_set_fmt,
	.get_selection = imx882_get_selection,
	.enable_streams = imx882_enable_streams,
	.disable_streams = imx882_disable_streams,
};

static const struct v4l2_subdev_ops imx882_subdev_ops = {
	.video = &imx882_video_ops,
	.pad = &imx882_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx882_internal_ops = {
	.init_state = imx882_init_state,
};

static int imx882_check_endpoint(struct imx882 *sensor)
{
	struct device *dev = sensor->dev;
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX882_LANES)
		ret = dev_err_probe(dev, -EINVAL, "need %u data lanes\n",
				    IMX882_LANES);
	else
		ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
					       bus_cfg.nr_of_link_frequencies,
					       imx882_link_freq,
					       ARRAY_SIZE(imx882_link_freq),
					       &sensor->link_freq_bitmap);

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int imx882_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx882 *sensor;
	u64 id;
	int ret, i;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->dev = dev;

	ret = imx882_check_endpoint(sensor);
	if (ret)
		return ret;
	sensor->mode = imx882_find_mode(sensor, 2048, 1536);

	sensor->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "cannot get clock\n");
	if (clk_get_rate(sensor->mclk) != IMX882_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "clock must be 24 MHz\n");

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = imx882_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get supplies\n");

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	v4l2_i2c_subdev_init(&sensor->sd, client, &imx882_subdev_ops);

	ret = imx882_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(sensor->regmap, IMX882_REG_CHIP_ID, &id, NULL);
	if (!ret && id != IMX882_CHIP_ID)
		ret = -ENODEV;
	if (ret) {
		dev_err_probe(dev, ret, "chip ID 0x%llx, expected 0x%x\n", id,
			      IMX882_CHIP_ID);
		goto err_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = imx882_init_controls(sensor);
	if (ret)
		goto err_pm;

	sensor->sd.internal_ops = &imx882_internal_ops;
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
	imx882_power_off(dev);
	return ret;
}

static void imx882_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx882 *sensor = to_imx882(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev)) {
		imx882_power_off(sensor->dev);
		pm_runtime_set_suspended(sensor->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx882_pm_ops, imx882_power_off,
				 imx882_power_on, NULL);

static const struct of_device_id imx882_of_match[] = {
	{ .compatible = "sony,imx882" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx882_of_match);

static struct i2c_driver imx882_driver = {
	.driver = {
		.name = "imx882",
		.of_match_table = imx882_of_match,
		.pm = pm_ptr(&imx882_pm_ops),
	},
	.probe = imx882_probe,
	.remove = imx882_remove,
};
module_i2c_driver(imx882_driver);

MODULE_DESCRIPTION("Sony IMX882 sensor driver");
MODULE_AUTHOR("Marcus Ramberg <marcus@means.no>");
MODULE_LICENSE("GPL");
