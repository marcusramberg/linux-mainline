// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC520CS 5 MP RAW sensor driver
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

#define SC520CS_LANES			2
#define SC520CS_BITS_PER_SAMPLE		10
#define SC520CS_LINK_FREQ		450000000
#define SC520CS_PIXEL_RATE		\
	(SC520CS_LINK_FREQ * 2 * SC520CS_LANES / SC520CS_BITS_PER_SAMPLE)
#define SC520CS_MCLK_FREQ		24000000
#define SC520CS_MBUS_CODE		MEDIA_BUS_FMT_SBGGR10_1X10

#define SC520CS_WIDTH			2592
#define SC520CS_HEIGHT			1944
/* 1500 sensor clocks at 90 MHz per line, 30 fps at VTS 2000 */
#define SC520CS_HTS			3000
#define SC520CS_VTS_DEF			2000
#define SC520CS_VTS_MAX			0x7fff

#define SC520CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC520CS_CHIP_ID			0xee4b

#define SC520CS_REG_MODE_SELECT		CCI_REG8(0x0100)
#define SC520CS_MODE_STANDBY		0x00
#define SC520CS_MODE_STREAMING		0x01

#define SC520CS_REG_VTS			CCI_REG16(0x320e)

/* half lines, 4 fractional bits */
#define SC520CS_REG_EXPOSURE		CCI_REG24(0x3e00)
#define SC520CS_EXPOSURE_MIN		2
#define SC520CS_EXPOSURE_MARGIN		5

#define SC520CS_REG_GROUP_HOLD		CCI_REG8(0x3812)
#define SC520CS_REG_COARSE_AGAIN	CCI_REG8(0x3e09)
#define SC520CS_REG_DGAIN_COARSE	CCI_REG8(0x3e06)
#define SC520CS_REG_DGAIN_FINE		CCI_REG8(0x3e07)
/* 1/1024 units: analogue steps of 2x, digital fine gain in between */
#define SC520CS_GAIN_MIN		1024
#define SC520CS_GAIN_MAX		(1024 * 635 / 10)

/* the sensor shifts its window by a pixel: the Bayer order stays BGGR */
#define SC520CS_REG_ORIENTATION		CCI_REG8(0x3221)
#define SC520CS_ORIENTATION_MIRROR	0x06
#define SC520CS_ORIENTATION_FLIP	0x60

#define SC520CS_REG_TEST_PATTERN	CCI_REG8(0x3337)
#define SC520CS_REG_TEST_PATTERN_2	CCI_REG8(0x391d)

static const char * const sc520cs_supply_names[] = {
	/* power-up order */
	"dovdd",
	"dvdd",
	"avdd",
};

static const struct cci_reg_sequence sc520cs_init_regs[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36e9), 0x23 },
	{ CCI_REG8(0x37f9), 0x23 },
	{ CCI_REG8(0x301f), 0x43 },
	{ CCI_REG8(0x3205), 0x27 },
	{ CCI_REG8(0x3209), 0x20 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3301), 0x08 },
	{ CCI_REG8(0x3306), 0x60 },
	{ CCI_REG8(0x3308), 0x10 },
	{ CCI_REG8(0x3309), 0x80 },
	{ CCI_REG8(0x330b), 0xd0 },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x331f), 0x71 },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x336c), 0xcf },
	{ CCI_REG8(0x3390), 0x09 },
	{ CCI_REG8(0x3391), 0x0f },
	{ CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0x0e },
	{ CCI_REG8(0x3394), 0x15 },
	{ CCI_REG8(0x3395), 0x19 },
	{ CCI_REG8(0x33ad), 0x1c },
	{ CCI_REG8(0x33af), 0x3f },
	{ CCI_REG8(0x33b3), 0x30 },
	{ CCI_REG8(0x33b5), 0x90 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x09 },
	{ CCI_REG8(0x34a7), 0x0b },
	{ CCI_REG8(0x34a8), 0x20 },
	{ CCI_REG8(0x34a9), 0x20 },
	{ CCI_REG8(0x34f8), 0x0f },
	{ CCI_REG8(0x34f9), 0x10 },
	{ CCI_REG8(0x3630), 0x88 },
	{ CCI_REG8(0x3633), 0x34 },
	{ CCI_REG8(0x3635), 0xc0 },
	{ CCI_REG8(0x3637), 0x52 },
	{ CCI_REG8(0x3638), 0xc1 },
	{ CCI_REG8(0x363a), 0x89 },
	{ CCI_REG8(0x3670), 0x4a },
	{ CCI_REG8(0x3674), 0x48 },
	{ CCI_REG8(0x3675), 0x28 },
	{ CCI_REG8(0x3676), 0x18 },
	{ CCI_REG8(0x367c), 0x0f },
	{ CCI_REG8(0x367d), 0x1f },
	{ CCI_REG8(0x3690), 0x43 },
	{ CCI_REG8(0x3691), 0x43 },
	{ CCI_REG8(0x3692), 0x43 },
	{ CCI_REG8(0x3698), 0x89 },
	{ CCI_REG8(0x3699), 0x92 },
	{ CCI_REG8(0x369a), 0xa5 },
	{ CCI_REG8(0x369b), 0xca },
	{ CCI_REG8(0x369c), 0x0b },
	{ CCI_REG8(0x369d), 0x0f },
	{ CCI_REG8(0x36a2), 0x09 },
	{ CCI_REG8(0x36a3), 0x0b },
	{ CCI_REG8(0x36a4), 0x0f },
	{ CCI_REG8(0x36b1), 0x38 },
	{ CCI_REG8(0x36b2), 0xc1 },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0xb1 },
	{ CCI_REG8(0x3771), 0x02 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x03 },
	{ CCI_REG8(0x377a), 0x0b },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3901), 0x04 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x391d), 0x01 },
	{ CCI_REG8(0x3926), 0x23 },
	{ CCI_REG8(0x3e00), 0x00 },
	{ CCI_REG8(0x3e01), 0xf9 },
	{ CCI_REG8(0x3e02), 0x70 },
	{ CCI_REG8(0x3e09), 0x00 },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0b },
	{ CCI_REG8(0x4404), 0x21 },
	{ CCI_REG8(0x4405), 0x29 },
	{ CCI_REG8(0x4407), 0x0c },
	{ CCI_REG8(0x440c), 0x34 },
	{ CCI_REG8(0x440d), 0x34 },
	{ CCI_REG8(0x440e), 0x29 },
	{ CCI_REG8(0x440f), 0x3f },
	{ CCI_REG8(0x4412), 0x01 },
	{ CCI_REG8(0x4424), 0x01 },
	{ CCI_REG8(0x442d), 0x00 },
	{ CCI_REG8(0x442e), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x19 },
	{ CCI_REG8(0x451d), 0x28 },
	{ CCI_REG8(0x4526), 0x05 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5007), 0x00 },
	{ CCI_REG8(0x550e), 0x00 },
	{ CCI_REG8(0x550f), 0x8a },
	{ CCI_REG8(0x5780), 0x66 },
	{ CCI_REG8(0x5787), 0x08 },
	{ CCI_REG8(0x5788), 0x04 },
	{ CCI_REG8(0x5789), 0x01 },
	{ CCI_REG8(0x578d), 0x40 },
	{ CCI_REG8(0x5790), 0x08 },
	{ CCI_REG8(0x5791), 0x04 },
	{ CCI_REG8(0x5792), 0x01 },
	{ CCI_REG8(0x5799), 0x46 },
	{ CCI_REG8(0x57aa), 0xeb },
	{ CCI_REG8(0x59f0), 0x00 },
	{ CCI_REG8(0x5ae0), 0xfe },
	{ CCI_REG8(0x5ae1), 0x40 },
	{ CCI_REG8(0x5ae2), 0x38 },
	{ CCI_REG8(0x5ae3), 0x30 },
	{ CCI_REG8(0x5ae4), 0x0c },
	{ CCI_REG8(0x5ae5), 0x38 },
	{ CCI_REG8(0x5ae6), 0x30 },
	{ CCI_REG8(0x5ae7), 0x28 },
	{ CCI_REG8(0x5ae8), 0x3f },
	{ CCI_REG8(0x5ae9), 0x34 },
	{ CCI_REG8(0x5aea), 0x2c },
	{ CCI_REG8(0x5aeb), 0x3f },
	{ CCI_REG8(0x5aec), 0x34 },
	{ CCI_REG8(0x5aed), 0x2c },
};

/* { gain threshold, 0x3e09 } */
static const u16 sc520cs_again[][2] = {
	{ 1024, 0x00 },
	{ 2048, 0x08 },
	{ 4096, 0x09 },
	{ 8192, 0x0b },
	{ 16384, 0x0f },
	{ 32768, 0x1f },
};

static const s64 sc520cs_link_freq[] = {
	SC520CS_LINK_FREQ,
};

static const char * const sc520cs_test_pattern_menu[] = {
	"Disabled",
	"Gradient",
};

struct sc520cs {
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
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(sc520cs_supply_names)];
};

#define to_sc520cs(_sd) container_of(_sd, struct sc520cs, sd)

static int sc520cs_power_on(struct device *dev)
{
	struct sc520cs *sensor = to_sc520cs(dev_get_drvdata(dev));
	int ret, i;

	ret = clk_prepare_enable(sensor->mclk);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	fsleep(3000);

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret)
			goto err_supplies;
		fsleep(1000);
	}

	gpiod_set_value_cansleep(sensor->powerdown_gpio, 0);
	fsleep(1000);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	fsleep(4000);

	return 0;

err_supplies:
	while (--i >= 0)
		regulator_disable(sensor->supplies[i].consumer);
	clk_disable_unprepare(sensor->mclk);
	return ret;
}

static int sc520cs_power_off(struct device *dev)
{
	struct sc520cs *sensor = to_sc520cs(dev_get_drvdata(dev));
	int i;

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	gpiod_set_value_cansleep(sensor->powerdown_gpio, 1);
	for (i = ARRAY_SIZE(sensor->supplies) - 1; i >= 0; i--)
		regulator_disable(sensor->supplies[i].consumer);
	clk_disable_unprepare(sensor->mclk);

	return 0;
}

static int sc520cs_set_gain(struct sc520cs *sensor, u32 gain)
{
	unsigned int i = ARRAY_SIZE(sc520cs_again) - 1;
	u32 fine;
	int ret = 0;

	while (i && gain < sc520cs_again[i][0])
		i--;
	/* 0x80 is 1x; at most 0xfe since GAIN_MAX < 2 * the top step */
	fine = gain * 128 / sc520cs_again[i][0];

	cci_write(sensor->regmap, SC520CS_REG_GROUP_HOLD, 0x00, &ret);
	cci_write(sensor->regmap, SC520CS_REG_COARSE_AGAIN,
		  sc520cs_again[i][1], &ret);
	cci_write(sensor->regmap, SC520CS_REG_DGAIN_COARSE, 0, &ret);
	cci_write(sensor->regmap, SC520CS_REG_DGAIN_FINE, fine, &ret);
	cci_write(sensor->regmap, SC520CS_REG_GROUP_HOLD, 0x30, &ret);

	return ret;
}

static int sc520cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc520cs *sensor = container_of(ctrl->handler, struct sc520cs,
					      ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int max = SC520CS_HEIGHT + ctrl->val - SC520CS_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       SC520CS_EXPOSURE_MIN, max, 1,
					       min(sensor->exposure->default_value,
						   (s64)max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, SC520CS_REG_EXPOSURE,
			  ctrl->val * 2 << 4, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc520cs_set_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, SC520CS_REG_VTS,
			  SC520CS_HEIGHT + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, SC520CS_REG_TEST_PATTERN,
			  ctrl->val ? 0x30 : 0x00, &ret);
		/* init writes 0x01 here, the vendor driver restores 0x19 */
		cci_write(sensor->regmap, SC520CS_REG_TEST_PATTERN_2,
			  ctrl->val ? 0x10 : 0x01, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, SC520CS_REG_ORIENTATION,
			  (sensor->hflip->val ? SC520CS_ORIENTATION_MIRROR : 0) |
			  (sensor->vflip->val ? SC520CS_ORIENTATION_FLIP : 0),
			  &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc520cs_ctrl_ops = {
	.s_ctrl = sc520cs_set_ctrl,
};

static int sc520cs_init_controls(struct sc520cs *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int vblank = SC520CS_VTS_DEF - SC520CS_HEIGHT;
	int exposure_max = SC520CS_VTS_DEF - SC520CS_EXPOSURE_MARGIN;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, SC520CS_PIXEL_RATE,
			  SC520CS_PIXEL_RATE, 1, SC520CS_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      sc520cs_link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
				 SC520CS_HTS - SC520CS_WIDTH,
				 SC520CS_HTS - SC520CS_WIDTH, 1,
				 SC520CS_HTS - SC520CS_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(hdl, &sc520cs_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   SC520CS_VTS_MAX - SC520CS_HEIGHT, 1,
					   vblank);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &sc520cs_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     SC520CS_EXPOSURE_MIN, exposure_max,
					     1, exposure_max);

	v4l2_ctrl_new_std(hdl, &sc520cs_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  SC520CS_GAIN_MIN, SC520CS_GAIN_MAX, 1,
			  SC520CS_GAIN_MIN);

	v4l2_ctrl_new_std_menu_items(hdl, &sc520cs_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(sc520cs_test_pattern_menu) - 1,
				     0, 0, sc520cs_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(hdl, &sc520cs_ctrl_ops, V4L2_CID_HFLIP,
					  0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(hdl, &sc520cs_ctrl_ops, V4L2_CID_VFLIP,
					  0, 1, 1, 0);

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &sc520cs_ctrl_ops, &props);

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

static int sc520cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = SC520CS_MBUS_CODE;

	return 0;
}

static int sc520cs_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != SC520CS_MBUS_CODE)
		return -EINVAL;

	fse->min_width = SC520CS_WIDTH;
	fse->max_width = SC520CS_WIDTH;
	fse->min_height = SC520CS_HEIGHT;
	fse->max_height = SC520CS_HEIGHT;

	return 0;
}

static int sc520cs_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	fmt->format = (struct v4l2_mbus_framefmt) {
		.width = SC520CS_WIDTH,
		.height = SC520CS_HEIGHT,
		.code = SC520CS_MBUS_CODE,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	return 0;
}

static int sc520cs_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect) { 0, 0, SC520CS_WIDTH,
					      SC520CS_HEIGHT };
		return 0;
	}

	return -EINVAL;
}

static int sc520cs_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct sc520cs *sensor = to_sc520cs(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, sc520cs_init_regs,
				  ARRAY_SIZE(sc520cs_init_regs), NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto err_put;

	ret = cci_write(sensor->regmap, SC520CS_REG_MODE_SELECT,
			SC520CS_MODE_STREAMING, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	pm_runtime_put_autosuspend(sensor->dev);
	return ret;
}

static int sc520cs_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct sc520cs *sensor = to_sc520cs(sd);
	int ret;

	ret = cci_write(sensor->regmap, SC520CS_REG_MODE_SELECT,
			SC520CS_MODE_STANDBY, NULL);
	pm_runtime_put_autosuspend(sensor->dev);

	return ret;
}

static int sc520cs_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_TRY };

	return sc520cs_set_fmt(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops sc520cs_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc520cs_pad_ops = {
	.enum_mbus_code = sc520cs_enum_mbus_code,
	.enum_frame_size = sc520cs_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sc520cs_set_fmt,
	.get_selection = sc520cs_get_selection,
	.enable_streams = sc520cs_enable_streams,
	.disable_streams = sc520cs_disable_streams,
};

static const struct v4l2_subdev_ops sc520cs_subdev_ops = {
	.video = &sc520cs_video_ops,
	.pad = &sc520cs_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc520cs_internal_ops = {
	.init_state = sc520cs_init_state,
};

static int sc520cs_check_endpoint(struct device *dev)
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != SC520CS_LANES)
		ret = dev_err_probe(dev, -EINVAL, "need %u data lanes\n",
				    SC520CS_LANES);
	else
		ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
					       bus_cfg.nr_of_link_frequencies,
					       sc520cs_link_freq,
					       ARRAY_SIZE(sc520cs_link_freq),
					       &(unsigned long){ 0 });

	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int sc520cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc520cs *sensor;
	u64 id;
	int ret, i;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->dev = dev;

	ret = sc520cs_check_endpoint(dev);
	if (ret)
		return ret;

	sensor->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "cannot get clock\n");
	if (clk_get_rate(sensor->mclk) != SC520CS_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "clock must be 24 MHz\n");

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "cannot get reset GPIO\n");

	sensor->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->powerdown_gpio),
				     "cannot get powerdown GPIO\n");

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = sc520cs_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get supplies\n");

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	v4l2_i2c_subdev_init(&sensor->sd, client, &sc520cs_subdev_ops);

	ret = sc520cs_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(sensor->regmap, SC520CS_REG_CHIP_ID, &id, NULL);
	if (!ret && id != SC520CS_CHIP_ID)
		ret = -ENODEV;
	if (ret) {
		dev_err_probe(dev, ret, "chip ID 0x%llx, expected 0x%x\n", id,
			      SC520CS_CHIP_ID);
		goto err_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = sc520cs_init_controls(sensor);
	if (ret)
		goto err_pm;

	sensor->sd.internal_ops = &sc520cs_internal_ops;
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
	sc520cs_power_off(dev);
	return ret;
}

static void sc520cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc520cs *sensor = to_sc520cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev)) {
		sc520cs_power_off(sensor->dev);
		pm_runtime_set_suspended(sensor->dev);
	}
}

static DEFINE_RUNTIME_DEV_PM_OPS(sc520cs_pm_ops, sc520cs_power_off,
				 sc520cs_power_on, NULL);

static const struct of_device_id sc520cs_of_match[] = {
	{ .compatible = "smartsens,sc520cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc520cs_of_match);

static struct i2c_driver sc520cs_driver = {
	.driver = {
		.name = "sc520cs",
		.of_match_table = sc520cs_of_match,
		.pm = pm_ptr(&sc520cs_pm_ops),
	},
	.probe = sc520cs_probe,
	.remove = sc520cs_remove,
};
module_i2c_driver(sc520cs_driver);

MODULE_DESCRIPTION("SmartSens SC520CS sensor driver");
MODULE_AUTHOR("Marcus Ramberg <marcus@means.no>");
MODULE_LICENSE("GPL");
