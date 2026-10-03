// SPDX-License-Identifier: GPL-2.0-only
/*
 * DJN FT8725 1080x2400 video mode panel (Motorola Moto G17).
 *
 * The bootloader powers the panel up and sends its init sequence; this driver
 * takes it over as is. Timings are the ones lk programs into the DSI host.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

struct ft8725 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	/* lk leaves it on; only wake it again after we put it to sleep */
	bool asleep;
};

static inline struct ft8725 *to_ft8725(struct drm_panel *panel)
{
	return container_of(panel, struct ft8725, panel);
}

static int ft8725_prepare(struct drm_panel *panel)
{
	struct ft8725 *ctx = to_ft8725(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (!ctx->asleep)
		return 0;

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	if (!dsi_ctx.accum_err)
		ctx->asleep = false;

	return dsi_ctx.accum_err;
}

static int ft8725_unprepare(struct drm_panel *panel)
{
	struct ft8725 *ctx = to_ft8725(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	ctx->asleep = true;

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode ft8725_mode = {
	/* lk's 1120 Mbit/s per lane, ~64 Hz */
	.clock = 186667,
	.hdisplay = 1080,
	.hsync_start = 1080 + 84,
	.hsync_end = 1080 + 84 + 10,
	.htotal = 1080 + 84 + 10 + 12,
	.vdisplay = 2400,
	.vsync_start = 2400 + 20,
	.vsync_end = 2400 + 20 + 10,
	.vtotal = 2400 + 20 + 10 + 20,
	.width_mm = 70,
	.height_mm = 156,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ft8725_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ft8725_mode);
}

static const struct drm_panel_funcs ft8725_panel_funcs = {
	.prepare = ft8725_prepare,
	.unprepare = ft8725_unprepare,
	.get_modes = ft8725_get_modes,
};

/*
 * 11 bits, sent as bits 10:3 then 2:0 (not the big endian u16 of
 * mipi_dsi_dcs_set_display_brightness_large()). The vendor driver tops out
 * at 1240 and only goes above in its HBM mode. Not the drm_panel's
 * backlight: drm_panel_enable() would send it right as video mode starts.
 */
static int ft8725_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 level = backlight_get_brightness(bl);
	u8 data[2] = { level >> 3, level & 0x7 };
	ssize_t ret;

	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				 data, sizeof(data));

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops ft8725_bl_ops = {
	.update_status = ft8725_bl_update_status,
};

static int ft8725_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1240,
		.max_brightness = 2047,
	};
	struct backlight_device *bl;
	struct ft8725 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ft8725, panel,
				   &ft8725_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					    &ft8725_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl), "Failed to register backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ft8725_remove(struct mipi_dsi_device *dsi)
{
	struct ft8725 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ft8725_of_match[] = {
	{ .compatible = "djn,ft8725" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft8725_of_match);

static struct mipi_dsi_driver ft8725_driver = {
	.probe = ft8725_probe,
	.remove = ft8725_remove,
	.driver = {
		.name = "panel-djn-ft8725",
		.of_match_table = ft8725_of_match,
	},
};
module_mipi_dsi_driver(ft8725_driver);

MODULE_DESCRIPTION("DRM driver for the DJN FT8725 video mode DSI panel");
MODULE_LICENSE("GPL");
