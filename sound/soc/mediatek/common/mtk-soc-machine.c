/*
* Copyright (C) 2015 MediaTek Inc.
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
#include "mtk-soc-codec-63xx.h"
#if defined(CONFIG_SND_SOC_RT5509) && defined(CONFIG_MTK_SMARTPA_SOUND)
#include <mt-plat/rt_nxp_compatible.h>
#endif

#ifdef CONFIG_SND_SOC_CS47L15
#include "../../codecs/madera.h"
#endif

static struct dentry *mt_sco_audio_debugfs;
#define DEBUG_FS_NAME "mtksocaudio"
#define DEBUG_ANA_FS_NAME "mtksocanaaudio"

#ifdef CONFIG_SND_SOC_CS47L15
struct gaines_clkdata {
	unsigned int sysclk_rate;
	unsigned int asyncclk_rate;
	unsigned int dspclk_rate;
};

struct gaines_clkdata gaines_clocks = {
	.sysclk_rate = 49152000,
	.asyncclk_rate = 0,
	.dspclk_rate = 0,
};

struct gaines_drvdata {
	struct device *dev;

	unsigned int pll_ref;
	unsigned int pll_out;
	unsigned int pll_source;
	unsigned int pll_id;
	enum snd_soc_bias_level prev_bias_level;
};

struct gaines_drvdata codec_data = {
	.pll_ref = 32768,
	.pll_source = MADERA_FLL_SRC_MCLK1,
	.pll_id = MADERA_FLL1_REFCLK,
	.pll_out = 147456000,
};

static const struct snd_soc_dapm_widget codec_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("HEADSETMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC1", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker", NULL),
};

static const struct snd_soc_dapm_route codec_dapm_routes[] = {
	{"HEADSETMIC", NULL, "MICBIAS1"},
	{"DMIC1", NULL, "MICBIAS1"},
	{"DMIC2", NULL, "MICBIAS2"},
	{"IN1L", NULL, "DMIC1"},
	{"IN1R", NULL, "DMIC1"},
	{"IN2L", NULL, "DMIC2"},
	{"IN2R", NULL, "DMIC2"},
	{"AIF1 Capture", NULL, "OPCLK"},
	{"AIF1 Playback", NULL, "OPCLK"},
};


static struct snd_soc_dai *find_codec_dai(struct snd_soc_card *card, const char *dai_name)
{
	int i;

	for (i = 0; i < card->num_rtd; i++) {
		if (!strcmp(card->rtd[i].codec_dai->name, dai_name)) {
			pr_err("$$VR: %s found %s dai\n", __func__, dai_name);
			return card->rtd[i].codec_dai;
		}
	}
	pr_err("+++VR: %s unable to find codec dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

static int codec_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	int ret;

	codec_dai = find_codec_dai(card, "cs47l15-aif1");
	if (codec_dai == NULL) {
		/* printk( KERN_ERR "Unable to find codec dai\n", __func__); */
		return 0;
	}

	codec = codec_dai->codec;

	ret = snd_soc_codec_set_pll(codec, MADERA_FLL1_REFCLK, MADERA_FLL_SRC_MCLK1, 0, 0);
	if (ret != 0) {
		pr_err("$$VR: %s Failed to enable FLL1 with Ref Clock Loop: %d\n",
		       __func__, ret);
		return ret;
	}
	/* SYS CLOCKS */
	ret = snd_soc_codec_set_sysclk(codec, MADERA_CLK_SYSCLK,
				       MADERA_CLK_SRC_FLL1, 49152000 * 2, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		pr_err("Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(codec, MADERA_CLK_OPCLK,
				       MADERA_CLK_SRC_FLL1, 12288000, SND_SOC_CLOCK_OUT);
	if (ret != 0) {
		pr_err("Failed to set OPCLK: %d\n", ret);
		return ret;
	}


	/* DSP */
/* if (largo_clocks.dspclk_rate) { */
	ret = snd_soc_codec_set_sysclk(codec, MADERA_CLK_DSPCLK,
				       MADERA_CLK_SRC_FLL1, 49152000 * 3, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		pr_err("Failed to set DSPCLK: %d\n", ret);
		return ret;
	}
/* } */

	return 0;
}

static int codec_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
/* struct largo_drvdata *data = card->drvdata; */
	struct snd_soc_dai *codec_dai;
	int ret;

	codec_dai = find_codec_dai(card, "cs47l15-aif1");

	if (codec_dai == NULL)
		return 0;

	codec = codec_dai->codec;
	if (dapm->dev != codec->dev)
		return 0;

/* printk( KERN_ERR "codec_set_bias_level level is %d data->prev_bias_level is %d\n", */
/* level,data->prev_bias_level); */

	switch (level) {

	case SND_SOC_BIAS_OFF:
		pr_err("$$VR : SND_SOC_STANDBY\n");
		/* if(data->prev_bias_level < SND_SOC_BIAS_PREPARE) */
		/* break; */

		ret = snd_soc_codec_set_pll(codec, MADERA_FLL1_REFCLK, MADERA_FLL_SRC_MCLK1, 0, 0);
		if (ret < 0) {
			pr_err("$$VR Failed to stop FLL: %d\n", ret);
			return ret;
		}
		ret = snd_soc_codec_set_sysclk(codec, MADERA_CLK_SYSCLK, MADERA_CLK_SRC_FLL1, 0, 0);
		if (ret < 0) {
			pr_err("$$VR Failed to set sysclk: %d\n", ret);
			return ret;
		}

		mdelay(1);
		break;

	default:
		break;
	}
	/* data->prev_bias_level = level; */
	return 0;
}

static int codec_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
/* struct largo_drvdata *data = card->drvdata; */
	struct snd_soc_dai *codec_dai;
	int ret;

	codec_dai = find_codec_dai(card, "cs47l15-aif3");

	if (codec_dai == NULL)
		return 0;

	codec = codec_dai->codec;

	if (dapm->dev != codec->dev)
		return 0;

/* printk( KERN_ERR "+++VR: level is %d, prev level=%d\n", */
/* level, data->prev_bias_level); */

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		pr_err("+++VR: SND_BIAS_PREPARE\n");
		if (dapm->bias_level != SND_SOC_BIAS_OFF)
			break;

		mdelay(1);
		ret = snd_soc_codec_set_pll(codec, MADERA_FLL1_REFCLK, MADERA_FLL_SRC_MCLK1,
					    2600000, 49152000 * 2);
		if (ret != 0) {
/* printk(KERN_ERR "+++VR: %s Failed to enable FLL1 with Ref Clock Loop: %d\n",__func__,ret); */
			return ret;
		}
		ret = snd_soc_codec_set_sysclk(codec, MADERA_CLK_SYSCLK,
					       MADERA_CLK_SRC_FLL1, 49152000 * 2, SND_SOC_CLOCK_IN);
		if (ret < 0) {
/* printk( KERN_ERR "+++VR: Failed to set sysclk: %d, %d\n", data->pll_out, ret); */
			return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int codec_aif1_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops codec_aif1_ops = {
/* .startup = codec_aif3_startup, */
	.hw_params = codec_aif1_hw_params,
/* .hw_free = codec_aif3_free, */
};

#endif

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
	char *buffer = NULL;	/* for reduce kernel stack */
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

static ssize_t mt_soc_debug_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	const int size = 6144;
	/* char buffer[size]; */
	char *buffer = NULL;	/* for reduce kernel stack */
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
		pr_warn("copy_from_user mt_soc_debug_write count = %zu, temp = %s\n", count, InputString);

	temp = kstrdup(InputString, GFP_KERNEL);
	pr_debug("copy_from_user mt_soc_debug_write count = %zu, temp = %s, pointer = %p\n",
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
		pr_debug("strcmp(token1, ParSetkeyAfe)\n");
		if ((token3 != NULL) && (token5 != NULL)) {
			ret = kstrtoul(token3, 16, &regaddr);
			ret = kstrtoul(token5, 16, &regvalue);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
			Afe_Set_Reg(regaddr, regvalue, 0xffffffff);
			regvalue = Afe_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 or token5 is NULL!\n");
		}
	}

	if (strcmp(token1, ParSetkeyAna) == 0) {
		pr_debug("strcmp(token1, ParSetkeyAna)\n");
		if ((token3 != NULL) && (token5 != NULL)) {
			ret = kstrtoul(token3, 16, &regaddr);
			ret = kstrtoul(token5, 16, &regvalue);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
			audckbufEnable(true);
			Ana_Set_Reg(regaddr, regvalue, 0xffffffff);
			regvalue = Ana_Get_Reg(regaddr);
			audckbufEnable(false);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 or token5 is NULL!\n");
		}
	}

	if (strcmp(token1, PareGetkeyAfe) == 0) {
		pr_debug("strcmp(token1, PareGetkeyAfe)\n");
		if (token3 != NULL) {
			ret = kstrtoul(token3, 16, &regaddr);
			regvalue = Afe_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", PareGetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 is NULL!\n");
		}
	}

	if (strcmp(token1, PareGetkeyAna) == 0) {
		pr_debug("strcmp(token1, PareGetkeyAna)\n");
		if (token3 != NULL) {
			ret = kstrtoul(token3, 16, &regaddr);
			regvalue = Ana_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", PareGetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 is NULL!\n");
		}
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
static int mt_machine_trigger(struct snd_pcm_substream *substream, int cmd)
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
#ifdef CONFIG_SND_SOC_CS47L15
	{
	 .name = "CS47L15 AIF1 Playback",
	 .stream_name = "Stream_Playback",
	 .cpu_dai_name = MT_SOC_I2S0DL1_NAME,
	 .platform_name = MT_SOC_I2S0DL1_PCM,
	 .codec_dai_name = "cs47l15-aif1",
	 .codec_name = "cs47l15-codec",
	 .ignore_suspend = 1,
	 .dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
	 .ops = &codec_aif1_ops,
	 },
	{
	 .name = "CS47L15 AIF1 Capture",
	 .stream_name = "Stream_Capture",
	 .cpu_dai_name = MT_SOC_I2S0AWBDAI_NAME,
	 .platform_name = MT_SOC_I2S0_AWB_PCM,
	 .codec_dai_name = "cs47l15-aif1",
	 .codec_name = "cs47l15-codec",
	 .ignore_suspend = 1,
	 .dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
	 .ops = &codec_aif1_ops,
	 },
#endif
	{
	 .name = "MultiMedia1",
	 .stream_name = MT_SOC_DL1_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_DL1DAI_NAME,
	 .platform_name = MT_SOC_DL1_PCM,
	 .codec_dai_name = MT_SOC_CODEC_TXDAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "MultiMedia2",
	 .stream_name = MT_SOC_UL1_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_UL1DAI_NAME,
	 .platform_name = MT_SOC_UL1_PCM,
	 .codec_dai_name = MT_SOC_CODEC_RXDAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "Voice_MD1",
	 .stream_name = MT_SOC_VOICE_MD1_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOICE_MD1_NAME,
	 .platform_name = MT_SOC_VOICE_MD1,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_MD1DAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
#ifdef _NON_COMMON_FEATURE_READY
	{
	 .name = "HDMI_OUT",
	 .stream_name = MT_SOC_HDMI_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_HDMI_NAME,
	 .platform_name = MT_SOC_HDMI_PCM,
	 .codec_dai_name = MT_SOC_CODEC_HDMI_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
#endif
	{
	 .name = "ULDLOOPBACK",
	 .stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_ULDLLOOPBACK_NAME,
	 .platform_name = MT_SOC_ULDLLOOPBACK_PCM,
	 .codec_dai_name = MT_SOC_CODEC_ULDLLOOPBACK_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "I2S0OUTPUT",
	 .stream_name = MT_SOC_I2S0_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_I2S0_NAME,
	 .platform_name = MT_SOC_I2S0_PCM,
	 .codec_dai_name = MT_SOC_CODEC_I2S0_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "MRGRX",
	 .stream_name = MT_SOC_MRGRX_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_MRGRX_NAME,
	 .platform_name = MT_SOC_MRGRX_PCM,
	 .codec_dai_name = MT_SOC_CODEC_MRGRX_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "MRGRXCAPTURE",
	 .stream_name = MT_SOC_MRGRX_CAPTURE_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_MRGRX_NAME,
	 .platform_name = MT_SOC_MRGRX_AWB_PCM,
	 .codec_dai_name = MT_SOC_CODEC_MRGRX_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "I2S0DL1OUTPUT",
	 .stream_name = MT_SOC_I2SDL1_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_I2S0DL1_NAME,
	 .platform_name = MT_SOC_I2S0DL1_PCM,
#ifdef CONFIG_SND_SOC_CS47L15
	 .codec_dai_name = "cs47l15-aif1",
	 .codec_name = "cs47l15-codec",
	 .ignore_suspend = 1,
	 .dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
	 .ops = &codec_aif1_ops,
#else
	 .codec_dai_name = MT_SOC_CODEC_I2S0TXDAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
#endif
	 },
	{
	 .name = "DL1AWBCAPTURE",
	 .stream_name = MT_SOC_DL1_AWB_RECORD_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_DL1AWB_NAME,
	 .platform_name = MT_SOC_DL1_AWB_PCM,
	 .codec_dai_name = MT_SOC_CODEC_DL1AWBDAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "Voice_MD1_BT",
	 .stream_name = MT_SOC_VOICE_MD1_BT_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOICE_MD1_BT_NAME,
	 .platform_name = MT_SOC_VOICE_MD1_BT,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_MD1_BTDAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "VOIP_CALL_BT_PLAYBACK",
	 .stream_name = MT_SOC_VOIP_BT_OUT_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOIP_CALL_BT_OUT_NAME,
	 .platform_name = MT_SOC_VOIP_BT_OUT,
	 .codec_dai_name = MT_SOC_CODEC_VOIPCALLBTOUTDAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "VOIP_CALL_BT_CAPTURE",
	 .stream_name = MT_SOC_VOIP_BT_IN_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOIP_CALL_BT_IN_NAME,
	 .platform_name = MT_SOC_VOIP_BT_IN,
	 .codec_dai_name = MT_SOC_CODEC_VOIPCALLBTINDAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "TDM_Debug_CAPTURE",
	 .stream_name = MT_SOC_TDM_CAPTURE_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_TDMRX_NAME,
	 .platform_name = MT_SOC_TDMRX_PCM,
	 .codec_dai_name = MT_SOC_CODEC_TDMRX_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "FM_MRG_TX",
	 .stream_name = MT_SOC_FM_MRGTX_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_FM_MRGTX_NAME,
	 .platform_name = MT_SOC_FM_MRGTX_PCM,
	 .codec_dai_name = MT_SOC_CODEC_FMMRGTXDAI_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "MultiMedia3",
	 .stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_UL2DAI_NAME,
	 .platform_name = MT_SOC_UL2_PCM,
	 .codec_dai_name = MT_SOC_CODEC_RXDAI2_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "I2S0_AWB_CAPTURE",
	 .stream_name = MT_SOC_I2S0AWB_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_I2S0AWBDAI_NAME,
	 .platform_name = MT_SOC_I2S0_AWB_PCM,
	 .codec_dai_name = MT_SOC_CODEC_I2S0AWB_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "Voice_MD2",
	 .stream_name = MT_SOC_VOICE_MD2_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOICE_MD2_NAME,
	 .platform_name = MT_SOC_VOICE_MD2,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_MD2DAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "PLATOFRM_CONTROL",
	 .stream_name = MT_SOC_ROUTING_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_ROUTING_DAI_NAME,
	 .platform_name = MT_SOC_ROUTING_PCM,
	 .codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "Voice_MD2_BT",
	 .stream_name = MT_SOC_VOICE_MD2_BT_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_VOICE_MD2_BT_NAME,
	 .platform_name = MT_SOC_VOICE_MD2_BT,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_MD2_BTDAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "HP_IMPEDANCE",
	 .stream_name = MT_SOC_HP_IMPEDANCE_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_HP_IMPEDANCE_NAME,
	 .platform_name = MT_SOC_HP_IMPEDANCE_PCM,
	 .codec_dai_name = MT_SOC_CODEC_HP_IMPEDANCE_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "FM_I2S_RX_Playback",
	 .stream_name = MT_SOC_FM_I2S_PLAYBACK_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_FM_I2S_NAME,
	 .platform_name = MT_SOC_FM_I2S_PCM,
	 .codec_dai_name = MT_SOC_CODEC_FM_I2S_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "FM_I2S_RX_Capture",
	 .stream_name = MT_SOC_FM_I2S_CAPTURE_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_FM_I2S_NAME,
	 .platform_name = MT_SOC_FM_I2S_AWB_PCM,
	 .codec_dai_name = MT_SOC_CODEC_FM_I2S_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
	{
	 .name = "MultiMedia_DL2",
	 .stream_name = MT_SOC_DL2_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_DL2DAI_NAME,
	 .platform_name = MT_SOC_DL2_PCM,
	 .codec_dai_name = MT_SOC_CODEC_TXDAI2_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
#ifdef CONFIG_MTK_BTCVSD_ALSA
	{
	 .name = "BTCVSD_RX",
	 .stream_name = MT_SOC_BTCVSD_CAPTURE_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = MT_SOC_BTCVSD_RX_PCM,
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
	 },
	{
	 .name = "BTCVSD_TX",
	 .stream_name = MT_SOC_BTCVSD_PLAYBACK_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = MT_SOC_BTCVSD_TX_PCM,
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
	 },
#endif
	{
	 .name = "MOD_DAI_CAPTURE",
	 .stream_name = MT_SOC_MODDAI_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_MOD_DAI_NAME,
	 .platform_name = MT_SOC_MOD_DAI_PCM,
	 .codec_dai_name = MT_SOC_CODEC_MOD_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 },
#ifdef _NON_COMMON_FEATURE_READY
	{
	 .name = "OFFLOAD",
	 .stream_name = MT_SOC_OFFLOAD_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = "snd-soc-dummy",
	 .codec_dai_name = MT_SOC_CODEC_OFFLOAD_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
#endif
#ifdef _NON_COMMON_FEATURE_READY
	{
	 .name = "PCM_ANC",
	 .stream_name = MT_SOC_ANC_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_ANC_NAME,
	 .platform_name = MT_SOC_ANC_PCM,
	 .codec_dai_name = MT_SOC_CODEC_ANC_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
#endif
	{
	 .name = "ANC_RECORD",
	 .stream_name = MT_SOC_ANC_RECORD_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_ANC_RECORD_DAI_NAME,
	 .platform_name = MT_SOC_I2S2_ADC2_PCM,
	 .codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_DUMMY_NAME,
	 .ops = &mt_machine_audio_ops,
	 },
#ifdef _NON_COMMON_FEATURE_READY
	{
	 .name = "Voice_Ultrasound",
	 .stream_name = MT_SOC_VOICE_ULTRA_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = MT_SOC_VOICE_ULTRA,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_ULTRADAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
#endif
	{
	 .name = "Voice_USB",
	 .stream_name = MT_SOC_VOICE_USB_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = MT_SOC_VOICE_USB,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_USBDAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 },
	{
	 .name = "Voice_USB_ECHOREF",
	 .stream_name = MT_SOC_VOICE_USB_ECHOREF_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = MT_SOC_VOICE_USB_ECHOREF,
	 .codec_dai_name = MT_SOC_CODEC_VOICE_USB_ECHOREF_DAI_NAME,
	 .codec_name = MT_SOC_CODEC_NAME,
	 .playback_only = true,
	 },
};

static struct snd_soc_dai_link mt_soc_extspk_dai[] = {
	{
	 .name = "ext_Speaker_Multimedia",
	 .stream_name = MT_SOC_SPEAKER_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = "snd-soc-dummy",
#ifdef CONFIG_SND_SOC_MAX98926
	 .codec_dai_name = "max98926-aif1",
	 .codec_name = "MAX98926_MT",
#elif defined(CONFIG_SND_SOC_RT5509)
	 .codec_dai_name = "rt5509-aif1",
	 .codec_name = "RT5509_MT_0",
	 .ignore_suspend = 1,
	 .ignore_pmdown_time = true,
#else
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
#endif
	 },
	{
	 .name = "I2S1_AWB_CAPTURE",
	 .stream_name = MT_SOC_I2S2ADC2_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_I2S2ADC2DAI_NAME,
	 .platform_name = MT_SOC_I2S2_ADC2_PCM,
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
	 .ops = &mt_machine_audio_ops,
	 },
};

#if defined(CONFIG_SND_SOC_RT5509) && defined(CONFIG_MTK_SMARTPA_SOUND)
static struct snd_soc_dai_link mt_soc_extspk_compatible_dai[] = {
	{
	 .name = "ext_Speaker_Multimedia",
	 .stream_name = MT_SOC_SPEAKER_STREAM_NAME,
	 .cpu_dai_name = "snd-soc-dummy-dai",
	 .platform_name = "snd-soc-dummy",
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
	 .ignore_suspend = 0,
	 .ignore_pmdown_time = 0,
	 },
	{
	 .name = "I2S1_AWB_CAPTURE",
	 .stream_name = MT_SOC_I2S2ADC2_STREAM_NAME,
	 .cpu_dai_name = MT_SOC_I2S2ADC2DAI_NAME,
	 .platform_name = MT_SOC_I2S2_ADC2_PCM,
	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name = "snd-soc-dummy",
	 .ops = &mt_machine_audio_ops,
	 },
};
#endif

static struct snd_soc_dai_link mt_soc_dai_component[ARRAY_SIZE(mt_soc_dai_common) +
						    ARRAY_SIZE(mt_soc_extspk_dai)];

static struct snd_soc_card mt_snd_soc_card_mt = {
	.name = "mt-snd-card",
	.dai_link = mt_soc_dai_common,
	.num_links = ARRAY_SIZE(mt_soc_dai_common),
#if defined(CONFIG_SND_SOC_LARGO) || defined(CONFIG_SND_SOC_CS47L15)
	.dapm_widgets = codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(codec_dapm_widgets),
	.dapm_routes = codec_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(codec_dapm_routes),
	.late_probe = codec_late_probe,
	.set_bias_level = codec_set_bias_level,
	.set_bias_level_post = codec_set_bias_level_post,
	.drvdata = &codec_data,
#endif
};

static int mt_soc_snd_init(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt_snd_soc_card_mt;
	int ret;
	int daiLinkNum = 0;

	pr_err("mt_soc_snd_init dai_link = %p\n", mt_snd_soc_card_mt.dai_link);

	/* DEAL WITH DAI LINK */
	memcpy(mt_soc_dai_component, mt_soc_dai_common, sizeof(mt_soc_dai_common));
	daiLinkNum += ARRAY_SIZE(mt_soc_dai_common);

#if defined(CONFIG_SND_SOC_RT5509) && defined(CONFIG_MTK_SMARTPA_SOUND)
	if (NXPExtSpk_i2c_client_registered()) {
		memcpy(mt_soc_dai_component + ARRAY_SIZE(mt_soc_dai_common),
		       mt_soc_extspk_compatible_dai, sizeof(mt_soc_extspk_compatible_dai));
		daiLinkNum += ARRAY_SIZE(mt_soc_extspk_compatible_dai);
	} else {
		memcpy(mt_soc_dai_component + ARRAY_SIZE(mt_soc_dai_common),
		       mt_soc_extspk_dai, sizeof(mt_soc_extspk_dai));
		daiLinkNum += ARRAY_SIZE(mt_soc_extspk_dai);
	}
#else
	memcpy(mt_soc_dai_component + ARRAY_SIZE(mt_soc_dai_common),
	       mt_soc_extspk_dai, sizeof(mt_soc_extspk_dai));
	daiLinkNum += ARRAY_SIZE(mt_soc_extspk_dai);
#endif

	mt_snd_soc_card_mt.dai_link = mt_soc_dai_component;
	mt_snd_soc_card_mt.num_links = daiLinkNum;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n", __func__, ret);

	/* create debug file */
	mt_sco_audio_debugfs = debugfs_create_file(DEBUG_FS_NAME,
						   S_IFREG | S_IRUGO, NULL, (void *)DEBUG_FS_NAME,
						   &mtaudio_debug_ops);

	/* create analog debug file */
	mt_sco_audio_debugfs = debugfs_create_file(DEBUG_ANA_FS_NAME,
						   S_IFREG | S_IRUGO, NULL,
						   (void *)DEBUG_ANA_FS_NAME,
						   &mtaudio_ana_debug_ops);

	return ret;
}

static const struct of_device_id mt_audio_driver_dt_match[] = {
	{.compatible = "mediatek,audio",},
	{}
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
