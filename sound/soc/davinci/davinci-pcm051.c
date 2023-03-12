/*
 * ASoC driver for phytec pcm051 development board (platform)
 *
 * Author:      Lars Poeschel <poeschel@lemonage.de>
 * Copyright:   (C) 2012 Lemonage Software GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <asm/hardware/asp.h>
#include <mach/edma.h>
#include <mach/board-pcm051.h>

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"
#include "../codecs/wm8974.h"

#define AUDIO_FORMAT (SND_SOC_DAIFMT_I2S | \
		SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF)

#define PCM051_AUXCLK 25000000

static int pcm051_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0, div, rate;
	unsigned sysclk;

	rate = params_rate(params);
	div = PCM051_AUXCLK / (rate * 16 * 2);
	ret = snd_soc_dai_set_clkdiv(cpu_dai, DAVINCI_MCASP_CLKXDIV,
			PCM051_AUXCLK / (rate * 16 * 2));
	if (ret < 0)
		return ret;

	if (div/3 > 31)
		sysclk = PCM051_AUXCLK/4;
	else if (div > 31)
		sysclk = PCM051_AUXCLK/3;
	else
		sysclk = PCM051_AUXCLK;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;

}

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* davinci-evm machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},

	/*  Speaker connected to (SPOP | SPOM) */
	{"SPOP", NULL, "Speaker"},
	{"SPOM", NULL, "Speaker"},
};
static struct snd_soc_ops pcm051_ops = {
	.hw_params = pcm051_hw_params,
};

static int pcm051_aic3007_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	if (machine_is_pcm051()) {
		snd_soc_dapm_disable_pin(dapm, "LINE2R");
		snd_soc_dapm_disable_pin(dapm, "LINE2L");
		snd_soc_dapm_enable_pin(dapm, "Speaker");
	}

	return 0;
}

static struct snd_soc_dai_link pcm051_dai_link = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.0",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-pcm-audio",
	.init = pcm051_aic3007_init,
	.ops = &pcm051_ops,
};

/* davinci pcm051 audio machine driver */
static struct snd_soc_card pcm051_snd_soc_card = {
	.name = "PCM051",
	.dai_link = &pcm051_dai_link,
	.num_links = 1,
};

static struct platform_device *pcm051_snd_device;

static int __init pcm051_init(void)
{
	struct snd_soc_card *pcm051_snd_dev_data;
	int index;
	int ret;

	pcm051_snd_dev_data = &pcm051_snd_soc_card;
	index = 0;

	pcm051_snd_device = platform_device_alloc("soc-audio", index);
	if (!pcm051_snd_device)
		return -ENOMEM;

	platform_set_drvdata(pcm051_snd_device, pcm051_snd_dev_data);
	ret = platform_device_add(pcm051_snd_device);
	if (ret)
		platform_device_put(pcm051_snd_device);

	return ret;
}

static void __exit pcm051_exit(void)
{
	platform_device_unregister(pcm051_snd_device);
}

module_init(pcm051_init);
module_exit(pcm051_exit);

MODULE_AUTHOR("Lars Poeschel");
MODULE_DESCRIPTION("Phytec PCM051 development board ASoC driver");
MODULE_LICENSE("GPL");
