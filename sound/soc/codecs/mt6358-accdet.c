// SPDX-License-Identifier: GPL-2.0
//
// mt6358-accdet.c  --  MT6358 headset jack detection
//
// Plug detection on the PMIC's EINT0 (polarity flipped after every edge),
// headset vs headphone from the ACCDET comparator state, buttons from the
// AUXADC voltage on MICBIAS1. Sequences follow MediaTek's vendor driver.

#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "mt6358-accdet.h"

#define MT6358_ACCDET_JACK_MASK	(SND_JACK_HEADSET | SND_JACK_BTN_0 | \
				 SND_JACK_BTN_1 | SND_JACK_BTN_2)

enum accdet_ab {
	AB_00 = 0,	/* MIC shorted: 3-pole plug or a button held */
	AB_01 = 1,	/* MIC biased: 4-pole headset */
	AB_11 = 3,	/* open */
};

/* "mediatek,pwm-deb-setting", stock order */
enum {
	PWM_WIDTH, PWM_THRESH, FALL_DELAY, RISE_DELAY,
	DEBOUNCE0, DEBOUNCE1, DEBOUNCE3, DEBOUNCE4,
	EINT_PWM_WIDTH, EINT_PWM_THRESH, EINT_DEBOUNCE0,
	PWM_DEB_NUM = 15,
};

struct mt6358_accdet {
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *auxadc;
	struct snd_soc_jack *jack;
	struct mutex lock;
	struct work_struct eint_work;
	struct work_struct accdet_work;

	u32 deb[PWM_DEB_NUM];
	u32 mic_vol;
	u32 eint_pol;
	u32 key_thr[3];		/* mid, up, down (mV) */

	bool plugged;
	bool mic_bias;		/* last AB was 01 */
	int jack_type;
	int btn_type;
	int key_mv;
};

static void accdet_irq_clear(struct mt6358_accdet *priv, u32 irq, u32 clr,
			     u32 status)
{
	unsigned int val;

	regmap_set_bits(priv->regmap, MT6358_ACCDET_CON12, clr);
	if (regmap_read_poll_timeout(priv->regmap, MT6358_ACCDET_CON12, val,
				     !(val & irq), 0, 400))
		dev_warn(priv->dev, "irq %#x stuck\n", irq);
	regmap_clear_bits(priv->regmap, MT6358_ACCDET_CON12, clr);
	regmap_write(priv->regmap, MT6358_AUD_TOP_INT_STATUS0, status);
}

static void accdet_report(struct mt6358_accdet *priv)
{
	if (priv->jack)
		snd_soc_jack_report(priv->jack, priv->jack_type | priv->btn_type,
				    MT6358_ACCDET_JACK_MASK);
}

static void accdet_seq_init(struct mt6358_accdet *priv)
{
	regmap_set_bits(priv->regmap, MT6358_ACCDET_CON1, ACCDET_SEQ_INIT);
	usleep_range(2000, 3000);
	regmap_clear_bits(priv->regmap, MT6358_ACCDET_CON1, ACCDET_SEQ_INIT);
	usleep_range(1000, 1500);

	/* debounce/32768 s */
	regmap_write(priv->regmap, MT6358_ACCDET_CON6, priv->deb[DEBOUNCE0]);
	regmap_write(priv->regmap, MT6358_ACCDET_CON7, priv->deb[DEBOUNCE1]);
	regmap_write(priv->regmap, MT6358_ACCDET_CON9, priv->deb[DEBOUNCE3]);
	regmap_write(priv->regmap, MT6358_ACCDET_CON10, priv->deb[DEBOUNCE4]);
}

static void accdet_set_eint_debounce(struct mt6358_accdet *priv, u32 deb)
{
	regmap_update_bits(priv->regmap, MT6358_ACCDET_CON15,
			   ACCDET_EINT0_DEB_MASK, deb << ACCDET_EINT0_DEB_SFT);
}

static void accdet_hw_init(struct mt6358_accdet *priv)
{
	accdet_seq_init(priv);
	accdet_set_eint_debounce(priv, priv->deb[EINT_DEBOUNCE0]);

	regmap_set_bits(priv->regmap, MT6358_AUD_TOP_RST_CON0, ACCDET_RST);
	regmap_clear_bits(priv->regmap, MT6358_AUD_TOP_RST_CON0, ACCDET_RST);

	regmap_write(priv->regmap, MT6358_ACCDET_CON3, priv->deb[PWM_WIDTH] - 1);
	regmap_write(priv->regmap, MT6358_ACCDET_CON4, priv->deb[PWM_THRESH] - 1);
	regmap_write(priv->regmap, MT6358_ACCDET_CON5,
		     priv->deb[FALL_DELAY] << 15 | priv->deb[RISE_DELAY]);

	/* MICBIAS1 level is ours; the codec only switches it on for AIN1 */
	regmap_set_bits(priv->regmap, MT6358_AUDENC_ANA_CON10,
			priv->mic_vol << MICBIAS1_VREF_SFT | MICBIAS1_LOWPEN);
	regmap_set_bits(priv->regmap, MT6358_AUDENC_ANA_CON11, ACCDET_MODE_ACC);

	if (priv->eint_pol == IRQ_TYPE_LEVEL_LOW)
		regmap_set_bits(priv->regmap, MT6358_AUDENC_ANA_CON6,
				AUDSPARE_FSTDSCHRG_IMPR |
				AUDSPARE_FSTDSCHRG_ANALOG_DIR);

	/* EINT0 comparator: the vendor's odd masks, kept bit for bit */
	regmap_update_bits(priv->regmap, MT6358_ACCDET_CON15, 0x6 << 8, 0x6 << 8);
	regmap_update_bits(priv->regmap, MT6358_ACCDET_CON15, 0x2 << 12, 0x2 << 12);
	regmap_set_bits(priv->regmap, MT6358_ACCDET_CON2,
			ACCDET_EINT0_PWM_EN | ACCDET_EINT0_PWM_IDLE);
	regmap_set_bits(priv->regmap, MT6358_AUDENC_ANA_CON11,
			ACCDET_EINT0_VTH_500K);

	regmap_update_bits(priv->regmap, MT6358_ACCDET_CON24,
			   ACCDET_HWMODE_SEL | ACCDET_FAST_DISCHARGE,
			   ACCDET_FAST_DISCHARGE);

	regmap_write(priv->regmap, MT6358_ACCDET_CON1, ACCDET_EINT0_EN);
}

static void accdet_enable(struct mt6358_accdet *priv)
{
	accdet_seq_init(priv);
	regmap_set_bits(priv->regmap, MT6358_ACCDET_CON2,
			ACCDET_CMP_PWM_IDLE | ACCDET_EINT0_PWM_IDLE |
			ACCDET_CMP_PWM_EN);
	regmap_set_bits(priv->regmap, MT6358_ACCDET_CON1, ACCDET_EN);
}

static void accdet_disable(struct mt6358_accdet *priv)
{
	regmap_clear_bits(priv->regmap, MT6358_ACCDET_CON2, ACCDET_CMP_PWM_IDLE);
	accdet_irq_clear(priv, ACCDET_IRQ, ACCDET_IRQ_CLR, INT_STATUS_ACCDET);
	regmap_write(priv->regmap, MT6358_ACCDET_CON6, priv->deb[DEBOUNCE0]);
	regmap_write(priv->regmap, MT6358_ACCDET_CON9, priv->deb[DEBOUNCE3]);
	regmap_clear_bits(priv->regmap, MT6358_ACCDET_CON1, ACCDET_EN);
	regmap_clear_bits(priv->regmap, MT6358_ACCDET_CON2, ACCDET_CMP_PWM_EN);
}

static int accdet_key(struct mt6358_accdet *priv)
{
	int mv = priv->key_mv;

	if (mv < priv->key_thr[0])
		return SND_JACK_BTN_0;
	if (mv < priv->key_thr[1])
		return SND_JACK_BTN_2;
	if (mv < priv->key_thr[2])
		return SND_JACK_BTN_1;
	return 0;
}

static void accdet_eint_work(struct work_struct *work)
{
	struct mt6358_accdet *priv =
		container_of(work, struct mt6358_accdet, eint_work);

	mutex_lock(&priv->lock);
	if (priv->plugged) {
		accdet_enable(priv);
	} else {
		accdet_disable(priv);
		priv->mic_bias = false;
		priv->jack_type = 0;
		priv->btn_type = 0;
		accdet_report(priv);
	}
	mutex_unlock(&priv->lock);
}

static void accdet_work(struct work_struct *work)
{
	struct mt6358_accdet *priv =
		container_of(work, struct mt6358_accdet, accdet_work);
	unsigned int val;
	int ab;

	mutex_lock(&priv->lock);
	regmap_read(priv->regmap, MT6358_ACCDET_CON14, &val);
	ab = (val >> ACCDET_STATE_AB_SFT) & ACCDET_STATE_AB_MASK;
	dev_dbg(priv->dev, "AB %d, %d mV\n", ab, priv->key_mv);

	if (!priv->plugged)
		goto out;

	switch (ab) {
	case AB_00:
		if (priv->mic_bias) {
			/* key down; an unplug also passes through AB=00 */
			usleep_range(10000, 12000);
			regmap_read(priv->regmap, MT6358_ACCDET_CON12, &val);
			if (!(val & ACCDET_EINT0_IRQ))
				priv->btn_type = accdet_key(priv);
		} else {
			priv->jack_type = SND_JACK_HEADPHONE;
		}
		break;
	case AB_01:
		if (!priv->mic_bias) {
			/* longer hook debounce so short presses still register */
			regmap_write(priv->regmap, MT6358_ACCDET_CON9,
				     priv->deb[DEBOUNCE3] * 30);
			regmap_write(priv->regmap, MT6358_ACCDET_CON6, 0x400);
		}
		priv->mic_bias = true;
		priv->jack_type = SND_JACK_HEADSET;
		priv->btn_type = 0;
		break;
	default:
		break;
	}
	accdet_report(priv);
out:
	mutex_unlock(&priv->lock);
}

static irqreturn_t mt6358_accdet_irq(int irq, void *data)
{
	struct mt6358_accdet *priv = data;
	unsigned int val;

	regmap_read(priv->regmap, MT6358_ACCDET_CON12, &val);

	if (val & ACCDET_EINT0_IRQ) {
		bool low = priv->eint_pol == IRQ_TYPE_LEVEL_LOW;

		priv->plugged = !priv->plugged;
		/* next edge is the opposite one */
		regmap_update_bits(priv->regmap, MT6358_ACCDET_CON12,
				   ACCDET_EINT0_IRQ_POLARITY,
				   priv->plugged == low ?
				   ACCDET_EINT0_IRQ_POLARITY : 0);
		accdet_irq_clear(priv, ACCDET_EINT0_IRQ, ACCDET_EINT0_IRQ_CLR,
				 INT_STATUS_ACCDET_EINT0);
		accdet_set_eint_debounce(priv, priv->plugged ?
					 ACCDET_EINT0_DEB_OUT_012 :
					 ACCDET_EINT0_DEB_IN_256);
		schedule_work(&priv->eint_work);
	} else if (val & ACCDET_IRQ) {
		if (priv->mic_bias && !IS_ERR_OR_NULL(priv->auxadc) &&
		    iio_read_channel_processed(priv->auxadc, &priv->key_mv) < 0)
			priv->key_mv = INT_MAX;
		accdet_irq_clear(priv, ACCDET_IRQ, ACCDET_IRQ_CLR,
				 INT_STATUS_ACCDET);
		schedule_work(&priv->accdet_work);
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

int mt6358_accdet_enable_jack_detect(struct snd_soc_component *component,
				     struct snd_soc_jack *jack)
{
	struct mt6358_accdet *priv = snd_soc_component_get_drvdata(component);

	if (jack) {
		snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
		snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEDOWN);
		snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	}

	mutex_lock(&priv->lock);
	priv->jack = jack;
	accdet_report(priv);
	mutex_unlock(&priv->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(mt6358_accdet_enable_jack_detect);

static const struct snd_soc_component_driver mt6358_accdet_component;

static void mt6358_accdet_cancel_work(void *data)
{
	struct mt6358_accdet *priv = data;

	cancel_work_sync(&priv->eint_work);
	cancel_work_sync(&priv->accdet_work);
}

static int mt6358_accdet_probe(struct platform_device *pdev)
{
	struct mt6397_chip *mt6397 = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mt6358_accdet *priv;
	u32 thr[4];
	int i, irq, ret;
	static const char * const irqs[] = { "accdet_irq", "accdet_eint0" };

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->regmap = mt6397->regmap;
	mutex_init(&priv->lock);
	INIT_WORK(&priv->eint_work, accdet_eint_work);
	INIT_WORK(&priv->accdet_work, accdet_work);

	if (of_property_read_u32_array(np, "mediatek,pwm-deb-setting",
				       priv->deb, PWM_DEB_NUM))
		return dev_err_probe(dev, -EINVAL, "no mediatek,pwm-deb-setting\n");
	if (of_property_read_u32(np, "mediatek,mic-vol", &priv->mic_vol))
		priv->mic_vol = 6;
	if (of_property_read_u32(np, "mediatek,eint-level-pol", &priv->eint_pol))
		priv->eint_pol = IRQ_TYPE_LEVEL_LOW;
	if (!of_property_read_u32_array(np, "mediatek,three-key-thr", thr, 4))
		memcpy(priv->key_thr, thr + 1, sizeof(priv->key_thr));

	/* buttons need it, plug detection does not */
	priv->auxadc = devm_iio_channel_get(dev, "accdet");
	if (IS_ERR(priv->auxadc)) {
		if (PTR_ERR(priv->auxadc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "no AUXADC channel, no buttons\n");
	}

	platform_set_drvdata(pdev, priv);
	ret = devm_snd_soc_register_component(dev, &mt6358_accdet_component,
					      NULL, 0);
	if (ret)
		return ret;

	accdet_hw_init(priv);

	/* devm frees the interrupts first, then cancels the work */
	ret = devm_add_action_or_reset(dev, mt6358_accdet_cancel_work, priv);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(irqs); i++) {
		irq = platform_get_irq_byname(pdev, irqs[i]);
		if (irq < 0)
			return irq;
		ret = devm_request_threaded_irq(dev, irq, NULL,
						mt6358_accdet_irq, IRQF_ONESHOT,
						irqs[i], priv);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct of_device_id mt6358_accdet_of_match[] = {
	{ .compatible = "mediatek,mt6358-accdet" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6358_accdet_of_match);

static struct platform_driver mt6358_accdet_driver = {
	.driver = {
		.name = "mt6358-accdet",
		.of_match_table = mt6358_accdet_of_match,
	},
	.probe = mt6358_accdet_probe,
};
module_platform_driver(mt6358_accdet_driver);

MODULE_DESCRIPTION("MediaTek MT6358 headset jack detection");
MODULE_LICENSE("GPL");
