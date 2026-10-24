// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple S5L-lineage SPI controller (A4 .. A13, "spi-1,samsung" in the ADT).
 *
 * Register layout from openiBoot (plat-a4/spi.c) and probing on T8030: the
 * block decodes 8 address bits, so the M1-era FIFOSTAT/IE/IF/SHIFTCFG
 * registers of spi-apple.c alias the low ones here. The TX FIFO only accepts
 * data while RUN is set.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define S5L_SPI_CTRL		0x000
#define S5L_SPI_CTRL_RUN	BIT(0)
#define S5L_SPI_CTRL_TX_RESET	BIT(2)
#define S5L_SPI_CTRL_RX_RESET	BIT(3)

#define S5L_SPI_CFG		0x004
#define S5L_SPI_CFG_RX_ONLY	BIT(0)
#define S5L_SPI_CFG_CPHA	BIT(1)
#define S5L_SPI_CFG_CPOL	BIT(2)
#define S5L_SPI_CFG_MASTER	BIT(3)
#define S5L_SPI_CFG_ENSCK	BIT(4)
#define S5L_SPI_CFG_LSB_FIRST	BIT(13)
#define S5L_SPI_CFG_WORD_SIZE	GENMASK(16, 15)

#define S5L_SPI_STATUS		0x008
#define S5L_SPI_STATUS_RX_DONE	BIT(0)
#define S5L_SPI_STATUS_TX_LEVEL	GENMASK(10, 6)
#define S5L_SPI_STATUS_RX_LEVEL	GENMASK(15, 11)
#define S5L_SPI_STATUS_TX_DONE	BIT(22)

#define S5L_SPI_PIN		0x00c
#define S5L_SPI_PIN_CS		BIT(1)

#define S5L_SPI_TXDATA		0x010
#define S5L_SPI_RXDATA		0x020
#define S5L_SPI_CLKDIV		0x030
#define S5L_SPI_CLKDIV_MAX	0x3ff
#define S5L_SPI_RXCNT		0x034
#define S5L_SPI_TXCNT		0x04c

/* ponytail: lock-step chunks sized to the smallest FIFO seen; no IRQ/DMA. */
#define S5L_SPI_CHUNK		16
#define S5L_SPI_TIMEOUT_US	100000

struct s5l_spi {
	void __iomem *regs;
	struct clk *clk;
};

static int s5l_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *dev,
				struct spi_transfer *t)
{
	struct s5l_spi *spi = spi_controller_get_devdata(ctlr);
	const u8 *tx = t->tx_buf;
	u8 *rx = t->rx_buf;
	u32 bpw = t->bits_per_word > 8 ? 2 : 1, words = t->len / bpw;
	u32 div, cfg, st, done = 0;
	int ret;

	div = DIV_ROUND_UP(clk_get_rate(spi->clk), t->speed_hz);
	div = clamp_t(u32, div, 1, S5L_SPI_CLKDIV_MAX + 1) - 1;
	cfg = S5L_SPI_CFG_MASTER | S5L_SPI_CFG_ENSCK |
	      (dev->mode & SPI_CPHA ? S5L_SPI_CFG_CPHA : 0) |
	      (dev->mode & SPI_CPOL ? S5L_SPI_CFG_CPOL : 0) |
	      (dev->mode & SPI_LSB_FIRST ? S5L_SPI_CFG_LSB_FIRST : 0) |
	      FIELD_PREP(S5L_SPI_CFG_WORD_SIZE, bpw - 1);

	writel(S5L_SPI_CTRL_RUN | S5L_SPI_CTRL_TX_RESET | S5L_SPI_CTRL_RX_RESET,
	       spi->regs + S5L_SPI_CTRL);
	writel(~0, spi->regs + S5L_SPI_STATUS);
	writel(div, spi->regs + S5L_SPI_CLKDIV);
	writel(cfg, spi->regs + S5L_SPI_CFG);
	writel(words, spi->regs + S5L_SPI_TXCNT);
	writel(words, spi->regs + S5L_SPI_RXCNT);
	writel(S5L_SPI_CTRL_RUN, spi->regs + S5L_SPI_CTRL);

	while (done < words) {
		u32 i, n = min_t(u32, S5L_SPI_CHUNK, words - done);

		for (i = 0; i < n; i++) {
			u32 v = 0;

			if (tx)
				v = bpw == 2 ? get_unaligned((const u16 *)tx + done + i) : tx[done + i];
			writel(v, spi->regs + S5L_SPI_TXDATA);
		}
		ret = readl_poll_timeout_atomic(spi->regs + S5L_SPI_STATUS, st,
						FIELD_GET(S5L_SPI_STATUS_RX_LEVEL, st) >= n,
						1, S5L_SPI_TIMEOUT_US);
		if (ret) {
			dev_err(&ctlr->dev, "timeout at %u/%u words, status %#x\n", done, words, st);
			break;
		}
		for (i = 0; i < n; i++) {
			u32 v = readl(spi->regs + S5L_SPI_RXDATA);

			if (rx && bpw == 2)
				put_unaligned(v, (u16 *)rx + done + i);
			else if (rx)
				rx[done + i] = v;
		}
		done += n;
	}
	if (!ret)
		ret = readl_poll_timeout_atomic(spi->regs + S5L_SPI_STATUS, st,
						st & S5L_SPI_STATUS_TX_DONE, 1, S5L_SPI_TIMEOUT_US);
	writel(~0, spi->regs + S5L_SPI_STATUS);
	return ret;
}

static void s5l_spi_set_cs(struct spi_device *dev, bool high)
{
	struct s5l_spi *spi = spi_controller_get_devdata(dev->controller);

	writel(high ? S5L_SPI_PIN_CS : 0, spi->regs + S5L_SPI_PIN);
}

static int s5l_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct s5l_spi *spi;
	int ret;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*spi));
	if (!ctlr)
		return -ENOMEM;
	spi = spi_controller_get_devdata(ctlr);

	spi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi->regs))
		return PTR_ERR(spi->regs);
	spi->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(spi->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(spi->clk), "bus clock\n");

	ctlr->bus_num = pdev->id;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	ctlr->set_cs = s5l_spi_set_cs;
	ctlr->transfer_one = s5l_spi_transfer_one;
	ctlr->use_gpio_descriptors = true;
	ctlr->auto_runtime_pm = true;
	ctlr->dev.of_node = pdev->dev.of_node;

	pm_runtime_set_active(&pdev->dev);
	ret = devm_pm_runtime_enable(&pdev->dev);
	if (ret < 0)
		return ret;

	writel(S5L_SPI_PIN_CS, spi->regs + S5L_SPI_PIN);
	writel(S5L_SPI_CFG_MASTER | S5L_SPI_CFG_ENSCK, spi->regs + S5L_SPI_CFG);
	writel(S5L_SPI_CTRL_RUN, spi->regs + S5L_SPI_CTRL);

	return devm_spi_register_controller(&pdev->dev, ctlr);
}

static const struct of_device_id s5l_spi_of_match[] = {
	{ .compatible = "apple,s5l-spi" },
	{}
};
MODULE_DEVICE_TABLE(of, s5l_spi_of_match);

static struct platform_driver s5l_spi_driver = {
	.probe = s5l_spi_probe,
	.driver = {
		.name = "apple-s5l-spi",
		.of_match_table = s5l_spi_of_match,
	},
};
module_platform_driver(s5l_spi_driver);

MODULE_DESCRIPTION("Apple S5L-lineage SPI controller driver");
MODULE_LICENSE("GPL");
