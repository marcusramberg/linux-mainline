/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MT6358_ACCDET_H_
#define _MT6358_ACCDET_H_

#include <sound/soc.h>

#define MT6358_AUD_TOP_RST_CON0		0x2220
#define   ACCDET_RST			BIT(1)
#define MT6358_AUD_TOP_INT_STATUS0	0x2234
#define   INT_STATUS_ACCDET		BIT(5)
#define   INT_STATUS_ACCDET_EINT0	BIT(6)

#define MT6358_AUDENC_ANA_CON6		0x2394
#define   AUDSPARE_FSTDSCHRG_ANALOG_DIR	BIT(5)
#define   AUDSPARE_FSTDSCHRG_IMPR	BIT(6)
#define MT6358_AUDENC_ANA_CON10		0x239c
#define   MICBIAS1_LOWPEN		BIT(2)
#define   MICBIAS1_VREF_SFT		4
#define MT6358_AUDENC_ANA_CON11		0x239e
#define   ACCDET_MODE_ACC		0x0807
#define   ACCDET_EINT0_VTH_500K		0x1c00

#define MT6358_ACCDET_CON1		0x250a
#define   ACCDET_EN			BIT(0)
#define   ACCDET_SEQ_INIT		BIT(1)
#define   ACCDET_EINT0_EN		BIT(2)
#define MT6358_ACCDET_CON2		0x250c
#define   ACCDET_CMP_PWM_EN		GENMASK(2, 0)
#define   ACCDET_EINT0_PWM_EN		BIT(3)
#define   ACCDET_CMP_PWM_IDLE		GENMASK(10, 8)
#define   ACCDET_EINT0_PWM_IDLE		BIT(11)
#define MT6358_ACCDET_CON3		0x250e	/* pwm width */
#define MT6358_ACCDET_CON4		0x2510	/* pwm thresh */
#define MT6358_ACCDET_CON5		0x2512	/* rise/fall delay */
#define MT6358_ACCDET_CON6		0x2514	/* debounce 0..4 */
#define MT6358_ACCDET_CON7		0x2516
#define MT6358_ACCDET_CON9		0x251a
#define MT6358_ACCDET_CON10		0x251c
#define MT6358_ACCDET_CON12		0x2520
#define   ACCDET_IRQ			BIT(0)
#define   ACCDET_EINT0_IRQ		BIT(2)
#define   ACCDET_IRQ_CLR		BIT(8)
#define   ACCDET_EINT0_IRQ_CLR		BIT(10)
#define   ACCDET_EINT0_IRQ_POLARITY	BIT(14)
#define MT6358_ACCDET_CON14		0x2524
#define   ACCDET_STATE_AB_SFT		6
#define   ACCDET_STATE_AB_MASK		0x3
#define MT6358_ACCDET_CON15		0x2526
#define   ACCDET_EINT0_DEB_SFT		3
#define   ACCDET_EINT0_DEB_MASK		(0xf << 3)
#define   ACCDET_EINT0_DEB_IN_256	0x0e
#define   ACCDET_EINT0_DEB_OUT_012	0x01
#define MT6358_ACCDET_CON24		0x2538
#define   ACCDET_HWMODE_SEL		BIT(2)
#define   ACCDET_FAST_DISCHARGE		BIT(4)

int mt6358_accdet_enable_jack_detect(struct snd_soc_component *component,
				     struct snd_soc_jack *jack);

#endif
