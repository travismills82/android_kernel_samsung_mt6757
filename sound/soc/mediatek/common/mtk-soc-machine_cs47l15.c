/*
* Copyright (C) 2017 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/


/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_machine.c
 *
 * Project:
 * --------
 *   Audio soc machine driver
 *
 * Description:
 * ------------
 *   Audio machine driver
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/
#define DEBUG

/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "mtk-auddrv-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"

#include "jack_madera_sysfs_cb.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/div64.h>
#include <stdarg.h>
#include <linux/module.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <linux/debugfs.h>
#include <sound/tlv.h>
#include <linux/mfd/madera/core.h>
#include <linux/extcon/extcon-madera.h>
#include "mtk-soc-codec-63xx.h"
#include "../../codecs/madera.h"
#include <sound/cirrus-amp-cal.h>
#if defined(CONFIG_SND_SOC_DBMDX)
#include <sound/dbmdx-export.h>
#endif

/* Used for debugging and test automation */
static struct dentry *mt_sco_audio_debugfs;
#define DEBUG_FS_NAME "mtksocaudio"
#define DEBUG_ANA_FS_NAME "mtksocanaaudio"

#define GAINES_BASECLK_48K	49152000
#define GAINES_BASECLK_44K1	45158400

#define GAINES_AMP_RATE	48000
#define GAINES_AMP_BCLK	(GAINES_AMP_RATE * 16 * 4)

#define GAINES_DAI_CODEC	0
#define GAINES_DAI_LEFT_AMP	2
#define GAINES_DAI_RIGHT_AMP	3
#define GAINES_DAI_MAX		5

/* Clock configuration */
#define SYSCLK_SRC      MADERA_CLK_SRC_FLL1
#define DSPCLK_SRC      MADERA_CLK_SRC_FLL1
#define ASYNCCLK_SRC    MADERA_CLK_SRC_FLL2
#define OUTCLK_SRC      MADERA_OUTCLK_ASYNCCLK

#define FLL1_FIN        26000000
#define FLL1_FOUT       GAINES_BASECLK_48K*2
#define FLL1_SRC        MADERA_FLL_SRC_MCLK1
#define FLL2_FIN        32768
#define FLL2_FOUT       GAINES_BASECLK_48K
#define FLL2_SRC        MADERA_FLL_SRC_MCLK2
#define FLL2_SYNC_SRC   MADERA_FLL_SRC_AIF1BCLK

struct gain_table {
	int min;	   /* Minimum impedance */
	int max;	   /* Maximum impedance */
	unsigned int gain; /* Register value to set for this measurement */
};

static struct impedance_table {
	struct gain_table hp_gain_table[5];
	char imp_region[4]; /* impedance region */
} imp_table = {
	.hp_gain_table = {
		{    0,      13,  0 },
		{   14,      26,  5 },
		{   27,      42,  9 },
		{   43,     100,  9 },
		{  101, INT_MAX, 12 },
	},
};
static DECLARE_TLV_DB_SCALE(digital_tlv, -6400, 50, 0);

struct clk_conf {
	int id;
	const char *name;
	int source;
	int rate;

	bool valid;
};

struct gaines_drvdata {
	struct device *dev;

	struct clk_conf fll1_refclk;
	struct clk_conf fll2_refclk;
	struct clk_conf fllao_refclk;
	struct clk_conf sysclk;
	struct clk_conf asyncclk;
	struct clk_conf dspclk;
	struct clk_conf opclk;
	struct clk_conf outclk;
	
	struct notifier_block nb;
	
	bool ear_mic;
	unsigned int hp_impedance_step;
};

static struct snd_soc_card mt_snd_soc_card_mt;

static struct gaines_drvdata gaines_drvdata;

static unsigned int baserate = GAINES_BASECLK_48K;

enum FLL_ID { FLL1, FLL2, FLL3, FLLAO };
enum CLK_ID { SYSCLK, ASYNCCLK, DSPCLK, OPCLK, OUTCLK };

static struct snd_soc_dai *gaines_get_codec_dai(struct snd_soc_card *card,
						int dai);
static int gaines_set_clock(struct snd_soc_card *card,
			    struct clk_conf *config);

static int map_fllid_with_name(const char *name)
{
	if (!strcmp(name, "fll1-refclk"))
		return FLL1;
	else if (!strcmp(name, "fll2-refclk"))
		return FLL2;
	else if (!strcmp(name, "fll3-refclk"))
		return FLL3;
	else if (!strcmp(name, "fllao-refclk"))
		return FLLAO;
	else
		return -1;
}

static int map_clkid_with_name(const char *name)
{
	if (!strcmp(name, "sysclk"))
		return SYSCLK;
	else if (!strcmp(name, "asyncclk"))
		return ASYNCCLK;
	else if (!strcmp(name, "dspclk"))
		return DSPCLK;
	else if (!strcmp(name, "opclk"))
		return OPCLK;
	else if (!strcmp(name, "outclk"))
		return OUTCLK;
	else
		return -1;
}

static int gaines_start_fll(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct gaines_drvdata *drvdata = card->drvdata;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	unsigned int fsrc = 0, fin = 0, fout = 0, pll_id;
	int ret;

	if (!config->valid)
		return 0;

	codec_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);
	codec = codec_dai->codec;

	pll_id = map_fllid_with_name(config->name);
	switch (pll_id) {
	case FLL1:
		fsrc = config->source;
		fin = config->rate;
		fout = drvdata->sysclk.rate;
		break;
	case FLL2:
	case FLLAO:
		fsrc = config->source;
		fin = config->rate;
		fout = drvdata->asyncclk.rate;
		break;
	default:
		dev_err(card->dev, "Unknown FLLID for %s\n", config->name);
	}

	dev_dbg(card->dev, "Setting %s fsrc=%d fin=%uHz fout=%uHz\n",
		config->name, fsrc, fin, fout);

	ret = snd_soc_codec_set_pll(codec, config->id, fsrc, fin, fout);
	if (ret)
		dev_err(card->dev, "Failed to start %s\n", config->name);

	return ret;
}

static int gaines_stop_fll(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	int ret;

	if (!config->valid)
		return 0;

	codec_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);
	codec = codec_dai->codec;

	ret = snd_soc_codec_set_pll(codec, config->id, 0, 0, 0);
	if (ret)
		dev_err(card->dev, "Failed to stop %s\n", config->name);

	return ret;
}

static int gaines_set_clock(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *aif1_dai;
	struct snd_soc_codec *codec;
	unsigned int freq = 0, clk_id;
	int ret;
	int dir = SND_SOC_CLOCK_IN;

	if (!config->valid)
		return 0;

	aif1_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);
	codec = aif1_dai->codec;

	clk_id = map_clkid_with_name(config->name);
	switch (clk_id) {
	case  SYSCLK:
		freq = config->rate;
		break;
	case ASYNCCLK:
		freq = config->rate;
		break;
	case DSPCLK:
		freq = config->rate;
		break;
	case OPCLK:
		freq = config->rate;
		dir = SND_SOC_CLOCK_OUT;
		break;
	case OUTCLK:
		freq = config->rate;
		break;
	default:
		dev_err(card->dev, "Unknown Clock ID for %s\n", config->name);
	}

	dev_dbg(card->dev, "Setting %s freq to %u Hz\n", config->name, freq);

	ret = snd_soc_codec_set_sysclk(codec, config->id,
				       config->source, freq, dir);
	if (ret)
		dev_err(card->dev, "Failed to set %s to %u Hz\n",
			config->name, freq);

	return ret;
}

static int gaines_stop_clock(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *aif1_dai;
	struct snd_soc_codec *codec;
	int ret;

	if (!config->valid)
		return 0;

	aif1_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);
	codec = aif1_dai->codec;

	ret = snd_soc_codec_set_sysclk(codec, config->id, 0, 0, 0);
	if (ret)
		dev_err(card->dev, "Failed to stop %s\n", config->name);

	return ret;
}

static int gaines_set_clocking(struct snd_soc_card *card,
				  struct gaines_drvdata *drvdata)
{
	int ret;

	ret = gaines_start_fll(card, &drvdata->fll1_refclk);
	if (ret)
		return ret;

	if (!drvdata->sysclk.rate) {
		ret = gaines_set_clock(card, &drvdata->sysclk);
		if (ret)
			return ret;
	}

	if (!drvdata->dspclk.rate) {
		ret = gaines_set_clock(card, &drvdata->dspclk);
		if (ret)
			return ret;
	}

	ret = gaines_set_clock(card, &drvdata->opclk);
	if (ret)
		return ret;

	return ret;
}

static struct snd_soc_dai *gaines_get_codec_dai(struct snd_soc_card *card,
						   int dai)
{
	struct snd_soc_pcm_runtime *rtd;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[dai].name);

	return rtd->codec_dai;
}

static int gaines_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct gaines_drvdata *drvdata = card->drvdata;
	unsigned int rate = params_rate(params);
	int ret;

	/* Treat sysclk rate zero as automatic mode */
	if (!drvdata->sysclk.rate) {
		if (rate % 4000)
			baserate = GAINES_BASECLK_44K1;
		else
			baserate = GAINES_BASECLK_48K;
	}

	dev_dbg(card->dev, "Requesting Rate: %dHz, FLL: %dHz\n", rate,
		drvdata->sysclk.rate ? drvdata->sysclk.rate : baserate * 2);

	/* Ensure we can't race against set_bias_level */
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	ret = gaines_set_clocking(card, drvdata);
	mutex_unlock(&card->dapm_mutex);

	return 0;
}


static struct snd_soc_ops gaines_ops = {
	.hw_params = gaines_hw_params,
};

static int gaines_read_clk_conf(struct device_node *np,
				   const char * const prop,
				   struct clk_conf *conf)
{
	u32 tmp;
	int ret;

	/* Truncate "cirrus," from prop_name to fetch clk_name */
	conf->name = &prop[7];

	ret = of_property_read_u32_index(np, prop, 0, &tmp);
	if (ret)
		return ret;

	conf->id = tmp;

	ret = of_property_read_u32_index(np, prop, 1, &tmp);
	if (ret)
		return ret;

	if (tmp < 0xffff)
		conf->source = tmp;
	else
		conf->source = -1;

	ret = of_property_read_u32_index(np, prop, 2, &tmp);
	if (ret)
		return ret;

	conf->rate = tmp;
	conf->valid = true;

	return 0;
}

static int __init get_impedance_region(char *str)
{
	/* Read model region */
	strncat(imp_table.imp_region, str, sizeof(imp_table.imp_region) - 1);

	return 0;
}
early_param("region1", get_impedance_region);

static int cs47l15_madera_put_impedance_volsw(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct snd_soc_card *card = codec->component.card;
	struct gaines_drvdata *drvdata = card->drvdata;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned int val, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	val += drvdata->hp_impedance_step;
	dev_info(card->dev,
			 "SET GAIN %d according to impedance, moved %d step\n",
			 val, drvdata->hp_impedance_step);

	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	return err;
}

void mt6757_madera_hpdet_cb(unsigned int meas)
{
	struct snd_soc_card *card = &mt_snd_soc_card_mt;
	struct gaines_drvdata *drvdata = card->drvdata;
	int jack_det;
	int i, num_hp_gain_table;

	if (meas == (INT_MAX / 100))
		jack_det = 0;
	else
		jack_det = 1;

	madera_jack_det = jack_det;

	dev_info(card->dev, "%s(%d) meas(%d)\n", __func__, jack_det, meas);

	num_hp_gain_table = (int) ARRAY_SIZE(imp_table.hp_gain_table);
	for (i = 0; i < num_hp_gain_table; i++) {
		if (meas < imp_table.hp_gain_table[i].min
				|| meas > imp_table.hp_gain_table[i].max)
			continue;

		dev_info(card->dev, "SET GAIN %d step for %d ohms\n",
			 imp_table.hp_gain_table[i].gain, meas);
		drvdata->hp_impedance_step = imp_table.hp_gain_table[i].gain;
	}
}

void mt6757_madera_micd_cb(bool mic)
{
	struct snd_soc_card *card = &mt_snd_soc_card_mt;
	struct gaines_drvdata *drvdata = card->drvdata;

	drvdata->ear_mic = mic;
	madera_ear_mic = mic;
	dev_info(card->dev, "%s: ear_mic = %d\n", __func__, drvdata->ear_mic);
}

void mt6757_madera_update_impedance_table(struct device_node *np)
{
	int len = ARRAY_SIZE(imp_table.hp_gain_table);
	u32 data[len * 3];
	int i, ret;
	char imp_str[14] = "imp_table";

	if (strlen(imp_table.imp_region) == 3) {
		strcat(imp_str, "_");
		strcat(imp_str, imp_table.imp_region);
	}

	if (of_find_property(np, imp_str, NULL))
		ret = of_property_read_u32_array(np, imp_str, data, (len * 3));
	else
		ret = of_property_read_u32_array(np, "imp_table", data,
								(len * 3));

	if (!ret) {
		pr_info("%s: data from DT\n", __func__);

		for (i = 0; i < len; i++) {
			imp_table.hp_gain_table[i].min = data[i * 3];
			imp_table.hp_gain_table[i].max = data[(i * 3) + 1];
			imp_table.hp_gain_table[i].gain = data[(i * 3) + 2];
		}
	}
}

static int mt6757_madera_notify(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	const struct madera_hpdet_notify_data *hp_inf;
	const struct madera_micdet_notify_data *md_inf;
	const struct gaines_drvdata *drvdata =
		container_of(nb, struct gaines_drvdata, nb);

	switch (event) {
	case MADERA_NOTIFY_VOICE_TRIGGER:
		break;
	case MADERA_NOTIFY_HPDET:
		hp_inf = data;
		mt6757_madera_hpdet_cb((hp_inf->impedance_x100 / 100));
		dev_info(drvdata->dev, "HPDET val=%d.%02d ohms\n",
			 hp_inf->impedance_x100 / 100,
			 hp_inf->impedance_x100 % 100);
		break;
	case MADERA_NOTIFY_MICDET:
		md_inf = data;
		mt6757_madera_micd_cb(md_inf->present);
		dev_info(drvdata->dev, "MICDET present=%c val=%d.%02d ohms\n",
			 md_inf->present ? 'Y' : 'N',
			 md_inf->impedance_x100 / 100,
			 md_inf->impedance_x100 % 100);
		break;
	default:
		dev_info(drvdata->dev, "notifier event=0x%lx data=0x%p\n",
			 event, data);
		break;
	}

	return NOTIFY_DONE;
}

static int gaines_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai;
	struct gaines_drvdata *drvdata = card->drvdata;
	int ret;

	codec_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_OFF)
			break;

		ret = gaines_set_clocking(card, drvdata);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	return 0;
}

static int gaines_set_bias_level_post(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai;
	struct gaines_drvdata *drvdata = card->drvdata;
	int ret;

	codec_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_OFF:
		ret = gaines_stop_fll(card, &drvdata->fll1_refclk);
		if (ret)
			return ret;

		if (!drvdata->sysclk.rate) {
			ret = gaines_stop_clock(card, &drvdata->sysclk);
			if (ret)
				return ret;
		}

		if (!drvdata->dspclk.rate) {
			ret = gaines_stop_clock(card, &drvdata->dspclk);
			if (ret)
				return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

#define SND_SOC_DAPM_SPK_S(wname, wsubseq, wevent) \
{	.id = snd_soc_dapm_spk, .name = wname, .kcontrol_news = NULL, \
	.num_kcontrols = 0, .subseq = wsubseq, .reg = SND_SOC_NOPM, \
	.event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD }

static int cs35l35_external_amp(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;

	dev_info(card->dev, "%s: %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		cirrus_amp_calib_apply();
		break;

	case SND_SOC_DAPM_PRE_PMD:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new cs47l15_codec_controls[] = {
	SOC_SINGLE_EXT_TLV("HPOUT1L Impedance Volume",
		MADERA_DAC_DIGITAL_VOLUME_1L,
		MADERA_OUT1L_VOL_SHIFT, 0xbf, 0,
		snd_soc_get_volsw, cs47l15_madera_put_impedance_volsw,
		digital_tlv),
	SOC_SINGLE_EXT_TLV("HPOUT1R Impedance Volume",
		MADERA_DAC_DIGITAL_VOLUME_1R,
		MADERA_OUT1L_VOL_SHIFT, 0xbf, 0,
		snd_soc_get_volsw, cs47l15_madera_put_impedance_volsw,
		digital_tlv),
};

static const struct snd_kcontrol_new codec_dapm_controls[] = {
	SOC_DAPM_PIN_SWITCH("ALWAYS MIC"),
	SOC_DAPM_PIN_SWITCH("RCV"),
	SOC_DAPM_PIN_SWITCH("SPK"),
};

static const struct snd_soc_dapm_widget codec_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("HEADSETMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC1", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_MIC("ALWAYS MIC", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker", NULL),
	SND_SOC_DAPM_OUTPUT("ALWAYS OUTPUT"),
	SND_SOC_DAPM_SPK_S("RCV", 1, cs35l35_external_amp),
	SND_SOC_DAPM_SPK_S("SPK", 1, cs35l35_external_amp),
};

#if defined(CONFIG_SND_SOC_CS35L35)
static struct snd_soc_codec_conf gaines_codec_conf[2] = {
	{
		.dev_name = "cs35l35.0-0040",
		.name_prefix = "Left",
	},
	{
		.dev_name = "cs35l35.0-0041",
		.name_prefix = "Right",
	},
};
#endif

static struct snd_soc_pcm_stream cs35l35_param = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 1,
};

static int mt_soc_ana_debug_open(struct inode *inode, struct file *file)
{
	pr_debug("mt_soc_ana_debug_open\n");
	return 0;
}

static ssize_t mt_soc_ana_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	const int size = 4096;
	/* char buffer[size]; */
	char *buffer = NULL; /* for reduce kernel stack */
	int n = 0;
	int ret = 0;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer) {
		kfree(buffer);
		return -ENOMEM;
	}

	pr_debug("mt_soc_ana_debug_read count = %zu\n", count);
	AudDrv_Clk_On();
	audckbufEnable(true);

	n = Ana_Debug_Read(buffer, size);

	pr_debug("mt_soc_ana_debug_read len = %d\n", n);

	audckbufEnable(false);
	AudDrv_Clk_Off();

	ret = simple_read_from_buffer(buf, count, pos, buffer, n);
	kfree(buffer);
	return ret;
}

static int mt_soc_debug_open(struct inode *inode, struct file *file)
{
	pr_debug("mt_soc_debug_open\n");
	return 0;
}

static ssize_t mt_soc_debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	const int size = 6144;
	/* char buffer[size]; */
	char *buffer = NULL; /* for reduce kernel stack */
	int n = 0;
	int ret = 0;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer) {
		kfree(buffer);
		return -ENOMEM;
	}

	AudDrv_Clk_On();

	n = AudDrv_Reg_Dump(buffer, size);
	pr_debug("mt_soc_debug_read len = %d\n", n);

	AudDrv_Clk_Off();

	ret = simple_read_from_buffer(buf, count, pos, buffer, n);
	kfree(buffer);
	return ret;
}

static char const ParSetkeyAfe[] = "Setafereg";
static char const ParSetkeyAna[] = "Setanareg";
static char const PareGetkeyAfe[] = "Getafereg";
static char const PareGetkeyAna[] = "Getanareg";

static ssize_t mt_soc_debug_write(struct file *f, const char __user *buf,
				  size_t count, loff_t *offset)
{
	int ret = 0;
	char InputString[256];
	char *token1 = NULL;
	char *token2 = NULL;
	char *token3 = NULL;
	char *token4 = NULL;
	char *token5 = NULL;
	char *temp = NULL;

	unsigned long regaddr = 0;
	unsigned long regvalue = 0;
	char delim[] = " ,";

	memset_io((void *)InputString, 0, 256);

	if (count > 256)
		count = 256;

	if (copy_from_user((InputString), buf, count))
		pr_warn("copy_from_user mt_soc_debug_write count = %zu temp = %s\n", count, InputString);

	temp = kstrdup(InputString, GFP_KERNEL);
	pr_debug("copy_from_user mt_soc_debug_write count = %zu temp = %s pointer = %p\n",
		count, InputString, InputString);
	token1 = strsep(&temp, delim);
	pr_debug("token1\n");
	pr_debug("token1 = %s\n", token1);
	token2 = strsep(&temp, delim);
	pr_debug("token2 = %s\n", token2);
	token3 = strsep(&temp, delim);
	pr_debug("token3 = %s\n", token3);
	token4 = strsep(&temp, delim);
	pr_debug("token4 = %s\n", token4);
	token5 = strsep(&temp, delim);
	pr_debug("token5 = %s\n", token5);

	if (strcmp(token1, ParSetkeyAfe) == 0) {
		pr_debug("strcmp (token1,ParSetkeyAfe)\n");
		ret = kstrtoul(token3, 16, &regaddr);
		ret = kstrtoul(token5, 16, &regvalue);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			ParSetkeyAfe, (unsigned int)regaddr,
			(unsigned int)regvalue);
		Afe_Set_Reg(regaddr,  regvalue, 0xffffffff);
		regvalue = Afe_Get_Reg(regaddr);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			ParSetkeyAfe, (unsigned int)regaddr,
			(unsigned int)regvalue);
	}
	if (strcmp(token1, ParSetkeyAna) == 0) {
		pr_debug("strcmp (token1,ParSetkeyAna)\n");
		ret = kstrtoul(token3, 16, &regaddr);
		ret = kstrtoul(token5, 16, &regvalue);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			ParSetkeyAna, (unsigned int)regaddr,
			(unsigned int)regvalue);
		audckbufEnable(true);
		Ana_Set_Reg(regaddr,  regvalue, 0xffffffff);
		regvalue = Ana_Get_Reg(regaddr);
		audckbufEnable(false);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			ParSetkeyAna, (unsigned int)regaddr,
			(unsigned int)regvalue);
	}
	if (strcmp(token1, PareGetkeyAfe) == 0) {
		pr_debug("strcmp (token1,PareGetkeyAfe)\n");
		ret =  kstrtoul(token3, 16, &regaddr);
		regvalue = Afe_Get_Reg(regaddr);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			PareGetkeyAfe, (unsigned int)regaddr,
			(unsigned int)regvalue);
	}
	if (strcmp(token1, PareGetkeyAna) == 0) {
		pr_debug("strcmp (token1,PareGetkeyAna)\n");
		ret =  kstrtoul(token3, 16, &regaddr);
		regvalue = Ana_Get_Reg(regaddr);
		pr_debug("%s regaddr = 0x%x regvalue = 0x%x\n",
			PareGetkeyAna, (unsigned int)regaddr,
			(unsigned int)regvalue);
	}
	return count;
}

static const struct file_operations mtaudio_debug_ops = {
	.open = mt_soc_debug_open,
	.read = mt_soc_debug_read,
	.write = mt_soc_debug_write,
};

static const struct file_operations mtaudio_ana_debug_ops = {
	.open = mt_soc_ana_debug_open,
	.read = mt_soc_ana_debug_read,
};

/* snd_soc_ops */
static int mt_machine_trigger(struct snd_pcm_substream *substream,
				     int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		EnableAfe(true);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		EnableAfe(false);
		return 0;
	}
	return -EINVAL;
}

static struct snd_soc_ops mt_machine_audio_ops = {
	.trigger = mt_machine_trigger,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt_soc_dai_common[] = {
	/* FrontEnd DAI Links */
	{
		.name = "C10 AIF1 Playback",
		.stream_name = "CS47L15_Playback",
		.cpu_dai_name   = MT_SOC_I2S0DL1_NAME,
		.platform_name  = MT_SOC_I2S0DL1_PCM,
		.codec_dai_name = "cs47l15-aif1",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
	{
		.name = "C10 AIF1 Capture",
		.stream_name = "CS47L15_Capture",
		.cpu_dai_name   = MT_SOC_I2S0AWBDAI_NAME,
		.platform_name  = MT_SOC_I2S0_AWB_PCM,
		.codec_dai_name = "cs47l15-aif1",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
#if defined(CONFIG_SND_SOC_CS35L35)
	{
		.name = "CS35L35_SPK",
		.stream_name = MT_SOC_SPEAKER_STREAM_NAME,
		.cpu_dai_name   = "cs47l15-aif2",
		.codec_dai_name = "cs35l35-pcm",
		.codec_name = "cs35l35.0-0040",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.params = &cs35l35_param,
	},
	{
		.name = "CS35L35_RCV",
		.stream_name = "CS35L35_AMP2_RCV",
		.cpu_dai_name   = "cs47l15-aif2",
		.codec_dai_name = "cs35l35-pcm",
		.codec_name = "cs35l35.0-0041",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.params = &cs35l35_param
	},
#endif
	{
		.name = "MultiMedia1",
		.stream_name = MT_SOC_DL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL1DAI_NAME,
		.platform_name  = MT_SOC_DL1_PCM,
		.codec_dai_name = MT_SOC_CODEC_TXDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "MultiMedia2",
		.stream_name = MT_SOC_UL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_UL1DAI_NAME,
		.platform_name  = MT_SOC_UL1_PCM,
		.codec_dai_name = "cs47l15-aif3",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
	{
		.name = "Voice_MD1",
		.stream_name = MT_SOC_VOICE_MD1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD1_NAME,
		.platform_name  = MT_SOC_VOICE_MD1,
		.codec_dai_name = "cs47l15-aif3",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "HDMI_OUT",
		.stream_name = MT_SOC_HDMI_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_HDMI_NAME,
		.platform_name  = MT_SOC_HDMI_PCM,
		.codec_dai_name = MT_SOC_CODEC_HDMI_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#endif
	{
		.name = "ULDLOOPBACK",
		.stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_ULDLLOOPBACK_NAME,
		.platform_name  = MT_SOC_ULDLLOOPBACK_PCM,
		.codec_dai_name = MT_SOC_CODEC_ULDLLOOPBACK_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "I2S0OUTPUT",
		.stream_name = MT_SOC_I2S0_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0_NAME,
		.platform_name  = MT_SOC_I2S0_PCM,
		.codec_dai_name = MT_SOC_CODEC_I2S0_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "MRGRX",
		.stream_name = MT_SOC_MRGRX_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_MRGRX_NAME,
		.platform_name  = MT_SOC_MRGRX_PCM,
		.codec_dai_name = MT_SOC_CODEC_MRGRX_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "MRGRXCAPTURE",
		.stream_name = MT_SOC_MRGRX_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_MRGRX_NAME,
		.platform_name  = MT_SOC_MRGRX_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_MRGRX_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "I2S0DL1OUTPUT",
		.stream_name = MT_SOC_I2SDL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0DL1_NAME,
		.platform_name  = MT_SOC_I2S0DL1_PCM,
		.codec_dai_name = "cs47l15-aif1",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
	{
		.name = "DL1AWBCAPTURE",
		.stream_name = MT_SOC_DL1_AWB_RECORD_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL1AWB_NAME,
		.platform_name  = MT_SOC_DL1_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_DL1AWBDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD1_BT",
		.stream_name = MT_SOC_VOICE_MD1_BT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD1_BT_NAME,
		.platform_name  = MT_SOC_VOICE_MD1_BT,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD1_BTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "VOIP_CALL_BT_PLAYBACK",
		.stream_name = MT_SOC_VOIP_BT_OUT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOIP_CALL_BT_OUT_NAME,
		.platform_name  = MT_SOC_VOIP_BT_OUT,
		.codec_dai_name = MT_SOC_CODEC_VOIPCALLBTOUTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "VOIP_CALL_BT_CAPTURE",
		.stream_name = MT_SOC_VOIP_BT_IN_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOIP_CALL_BT_IN_NAME,
		.platform_name  = MT_SOC_VOIP_BT_IN,
		.codec_dai_name = MT_SOC_CODEC_VOIPCALLBTINDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "TDM_Debug_CAPTURE",
		.stream_name = MT_SOC_TDM_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_TDMRX_NAME,
		.platform_name  = MT_SOC_TDMRX_PCM,
		.codec_dai_name = MT_SOC_CODEC_TDMRX_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "FM_MRG_TX",
		.stream_name = MT_SOC_FM_MRGTX_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_MRGTX_NAME,
		.platform_name  = MT_SOC_FM_MRGTX_PCM,
		.codec_dai_name = MT_SOC_CODEC_FMMRGTXDAI_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "MultiMedia3",
		.stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_UL2DAI_NAME,
		.platform_name  = MT_SOC_UL2_PCM,
		.codec_dai_name = MT_SOC_CODEC_RXDAI2_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "I2S0_AWB_CAPTURE",
		.stream_name = MT_SOC_I2S0AWB_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0AWBDAI_NAME,
		.platform_name  = MT_SOC_I2S0_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_I2S0AWB_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD2",
		.stream_name = MT_SOC_VOICE_MD2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD2_NAME,
		.platform_name  = MT_SOC_VOICE_MD2,
		.codec_dai_name = "cs47l15-aif3",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
	{
		.name = "PLATOFRM_CONTROL",
		.stream_name = MT_SOC_ROUTING_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_ROUTING_DAI_NAME,
		.platform_name  = MT_SOC_ROUTING_PCM,
		.codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD2_BT",
		.stream_name = MT_SOC_VOICE_MD2_BT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD2_BT_NAME,
		.platform_name  = MT_SOC_VOICE_MD2_BT,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD2_BTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "HP_IMPEDANCE",
		.stream_name = MT_SOC_HP_IMPEDANCE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_HP_IMPEDANCE_NAME,
		.platform_name  = MT_SOC_HP_IMPEDANCE_PCM,
		.codec_dai_name = MT_SOC_CODEC_HP_IMPEDANCE_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "FM_I2S_RX_Playback",
		.stream_name = MT_SOC_FM_I2S_PLAYBACK_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_I2S_NAME,
		.platform_name  = MT_SOC_FM_I2S_PCM,
		.codec_dai_name = "cs47l15-aif1",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
	{
		.name = "FM_I2S_RX_Capture",
		.stream_name = MT_SOC_FM_I2S_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_I2S_NAME,
		.platform_name  = MT_SOC_FM_I2S_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_FM_I2S_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "MultiMedia_DL2",
		.stream_name = MT_SOC_DL2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL2DAI_NAME,
		.platform_name  = MT_SOC_DL2_PCM,
		.codec_dai_name = "cs47l15-aif1",
		.codec_name = "cs47l15-codec",
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &gaines_ops,
	},
#ifdef CONFIG_MTK_BTCVSD_ALSA
	{
		.name = "BTCVSD_RX",
		.stream_name = MT_SOC_BTCVSD_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = MT_SOC_BTCVSD_RX_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "BTCVSD_TX",
		.stream_name = MT_SOC_BTCVSD_PLAYBACK_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = MT_SOC_BTCVSD_TX_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
	{
		.name = "MOD_DAI_CAPTURE",
		.stream_name = MT_SOC_MODDAI_STREAM_NAME,
		.cpu_dai_name	= MT_SOC_MOD_DAI_NAME,
		.platform_name	= MT_SOC_MOD_DAI_PCM,
		.codec_dai_name = MT_SOC_CODEC_MOD_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "OFFLOAD",
		.stream_name = MT_SOC_OFFLOAD_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= "snd-soc-dummy",
		.codec_dai_name = MT_SOC_CODEC_OFFLOAD_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "PCM_ANC",
		.stream_name = MT_SOC_ANC_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_ANC_NAME,
		.platform_name  = MT_SOC_ANC_PCM,
		.codec_dai_name = MT_SOC_CODEC_ANC_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
	{
		.name = "ANC_RECORD",
		.stream_name = MT_SOC_ANC_RECORD_STREAM_NAME,
		.cpu_dai_name	= MT_SOC_ANC_RECORD_DAI_NAME,
		.platform_name	= MT_SOC_I2S2_ADC2_PCM,
		.codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
		.ops = &mt_machine_audio_ops,
	},
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "Voice_Ultrasound",
		.stream_name = MT_SOC_VOICE_ULTRA_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_ULTRA,
		.codec_dai_name = MT_SOC_CODEC_VOICE_ULTRADAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
	{
		.name = "Voice_USB",
		.stream_name = MT_SOC_VOICE_USB_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_USB,
		.codec_dai_name = MT_SOC_CODEC_VOICE_USBDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "Voice_USB_ECHOREF",
		.stream_name = MT_SOC_VOICE_USB_ECHOREF_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_USB_ECHOREF,
		.codec_dai_name = MT_SOC_CODEC_VOICE_USB_ECHOREF_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
		.playback_only = true,
	},
};

#if defined(CONFIG_SND_SOC_CS35L35)
static int gaines_amp_late_probe(struct snd_soc_card *card, int dai)
{
	struct gaines_drvdata *drvdata = card->drvdata;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *amp_dai;
	struct snd_soc_codec *amp;
	struct madera *madera;
	int ret;

	if (!card->dai_link[dai].name)
		return 0;

	if (!drvdata->opclk.valid) {
		dev_err(card->dev, "OPCLK required to use speaker amp\n");
		return -ENOENT;
	}

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[dai].name);

	madera = dev_get_drvdata(rtd->cpu_dai->codec->dev->parent);

	amp_dai = rtd->codec_dai;
	amp = amp_dai->codec;

	dev_dbg(card->dev, "%s tdm slot setting\n", __func__);
	
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 4, 16);
	if (ret)
		dev_err(card->dev, "Failed to set TDM: %d\n", ret);
	
	ret = snd_soc_codec_set_sysclk(amp, 0, 0, drvdata->opclk.rate,
				       SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(card->dev, "Failed to set amp SYSCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(amp_dai, 0, GAINES_AMP_BCLK,
				     SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(card->dev, "Failed to set amp DAI clock: %d\n", ret);
		return ret;
	}

	cirrus_amp_calib_set_regmap(madera->regmap_32bit);

	return 0;
}
#endif

static int gaines_late_probe(struct snd_soc_card *card)
{
	struct gaines_drvdata *drvdata = card->drvdata;
	struct snd_soc_dai *aif1_dai;
	struct snd_soc_codec *codec;
	int ret;

	aif1_dai = gaines_get_codec_dai(card, GAINES_DAI_CODEC);
	codec = aif1_dai->codec;

	ret = snd_soc_dai_set_sysclk(aif1_dai, drvdata->sysclk.id, 0, 0);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	if (drvdata->sysclk.rate) {
		ret = gaines_set_clock(card, &drvdata->sysclk);
		if (ret)
			return ret;
	}

	if (drvdata->dspclk.rate) {
		ret = gaines_set_clock(card, &drvdata->dspclk);
		if (ret)
			return ret;
	}

	ret = gaines_set_clock(card, &drvdata->asyncclk);
	if (ret)
		return ret;

	ret = gaines_set_clock(card, &drvdata->outclk);
	if (ret)
		return ret;

#if defined(CONFIG_SND_SOC_CS35L35)
	ret = gaines_amp_late_probe(card, GAINES_DAI_LEFT_AMP);
	if (ret)
		return ret;

	ret = gaines_amp_late_probe(card, GAINES_DAI_RIGHT_AMP);
	if (ret)
		return ret;
#endif

	ret = snd_soc_add_codec_controls(codec, cs47l15_codec_controls,
					ARRAY_SIZE(cs47l15_codec_controls));
	if (ret < 0) {
		dev_err(card->dev,
			"Failed to add controls to codec: %d\n", ret);
		return ret;
	}

	drvdata->nb.notifier_call = mt6757_madera_notify;
	madera_register_notifier(codec, &drvdata->nb);

	register_madera_jack_cb(codec);

#if defined(CONFIG_SND_SOC_DBMDX)
	/* dbmd control add will be move to machine driver */
	dbmdx_remote_add_codec_controls(codec);
#endif

	return 0;
}

static struct snd_soc_card mt_snd_soc_card_mt = {
	.name       = "mt-snd-card",
	.dai_link   = mt_soc_dai_common,
	.num_links  = ARRAY_SIZE(mt_soc_dai_common),
	.controls = codec_dapm_controls,
	.num_controls = ARRAY_SIZE(codec_dapm_controls),
	.dapm_widgets = codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(codec_dapm_widgets),
#if defined(CONFIG_SND_SOC_CS35L35)
	.codec_conf = gaines_codec_conf,
	.num_configs = ARRAY_SIZE(gaines_codec_conf),
#endif
	.late_probe = gaines_late_probe,
	.set_bias_level = gaines_set_bias_level,
	.set_bias_level_post = gaines_set_bias_level_post,
	.drvdata = &gaines_drvdata,
};

static int mt_soc_snd_init(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt_snd_soc_card_mt;
	struct gaines_drvdata *drvdata = card->drvdata;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	pr_err("mt_soc_snd_init dai_link = %p\n", mt_snd_soc_card_mt.dai_link);

	card->dev = &pdev->dev;
	drvdata->dev = card->dev;

	ret = gaines_read_clk_conf(np, "cirrus,sysclk", &drvdata->sysclk);
	if (ret) {
		dev_err(card->dev, "Failed to parse sysclk: %d\n", ret);
		return ret;
	}
	ret = gaines_read_clk_conf(np, "cirrus,asyncclk",
					  &drvdata->asyncclk);
	if (ret)
		dev_info(card->dev, "Failed to parse asyncclk: %d\n", ret);
		
	ret = gaines_read_clk_conf(np, "cirrus,dspclk", &drvdata->dspclk);
	if (ret) {
		dev_info(card->dev, "Failed to parse dspclk: %d\n", ret);
	} else if (drvdata->dspclk.source == drvdata->sysclk.source &&
		   drvdata->dspclk.rate / 3 != drvdata->sysclk.rate / 2) {
		/* DSPCLK & SYSCLK, if sharing a source, should be an
		 * appropriate ratio of one another, or both be zero (which
		 * signifies "automatic" mode).
		 */
		dev_err(card->dev, "DSPCLK & SYSCLK share src but request incompatible frequencies: %d vs %d\n",
			drvdata->dspclk.rate, drvdata->sysclk.rate);
		return -EINVAL;
	}
		
	ret = gaines_read_clk_conf(np, "cirrus,opclk", &drvdata->opclk);
	if (ret)
		dev_info(card->dev, "Failed to parse opclk: %d\n", ret);
		
	ret = gaines_read_clk_conf(np, "cirrus,fll1-refclk",
					  &drvdata->fll1_refclk);
	if (ret)
		dev_info(card->dev, "Failed to parse fll1-refclk: %d\n", ret);
		
	ret = gaines_read_clk_conf(np, "cirrus,fll2-refclk",
					  &drvdata->fll2_refclk);
	if (ret)
		dev_info(card->dev, "Failed to parse fll2-refclk: %d\n", ret);
		
	ret = gaines_read_clk_conf(np, "cirrus,fllao-refclk",
					  &drvdata->fllao_refclk);
	if (ret)
		dev_info(card->dev, "Failed to parse fllao-refclk: %d\n", ret);
		
	ret = gaines_read_clk_conf(np, "cirrus,outclk", &drvdata->outclk);
	if (ret)
		dev_info(card->dev, "Failed to parse outclk: %d\n", ret);

	ret = snd_soc_of_parse_audio_routing(card, "mediatek,audio-routing");
	if (ret) {
		dev_err(card->dev, "Failed to parse audio routing %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	mt6757_madera_update_impedance_table(np);

	/* create debug file */
	mt_sco_audio_debugfs =
		debugfs_create_file(DEBUG_FS_NAME, S_IFREG | S_IRUGO, NULL,
				    (void *) DEBUG_FS_NAME,
				    &mtaudio_debug_ops);

	/* create analog debug file */
	mt_sco_audio_debugfs =
		debugfs_create_file(DEBUG_ANA_FS_NAME, S_IFREG | S_IRUGO, NULL,
				    (void *) DEBUG_ANA_FS_NAME,
				    &mtaudio_ana_debug_ops);

#if defined(CONFIG_SND_SOC_CS47L15)
	pr_info("%s: enable 32k clk src in TOP_CKPDN_CON2(0x40C)\n", __func__);
	Ana_Set_Reg(0x40C, 0x0000, 0x2000);
#endif

	return ret;
}

static const struct of_device_id mt_audio_driver_dt_match[] = {
	{ .compatible = "mediatek,audio", },
	{ }
};

MODULE_DEVICE_TABLE(of, mt_audio_driver_dt_match);

static struct platform_driver mt_audio_driver = {
	.driver = {
		   .name = "mtk-audio",
		   .of_match_table = mt_audio_driver_dt_match,
	},
	.probe = mt_soc_snd_init,
};

module_platform_driver(mt_audio_driver);

/* Module information */
MODULE_AUTHOR("ChiPeng <chipeng.chang@mediatek.com>");
MODULE_DESCRIPTION("ALSA SoC driver ");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mt-snd-card");
