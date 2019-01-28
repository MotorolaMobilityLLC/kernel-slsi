/*
 *  Driver for Madera CODECs on Exynos9610
 *
 *  Copyright 2013 Wolfson Microelectronics
 *  Copyright 2016 Cirrus Logic
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <linux/mfd/madera/core.h>
#include <linux/extcon/extcon-madera.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <soc/samsung/exynos-pmu.h>
#include <sound/samsung/abox.h>

#if IS_ENABLED(CONFIG_SND_SOC_MADERA)
#include <linux/mfd/madera/core.h>
#include <linux/extcon/extcon-madera.h>
#include "../codecs/madera.h"
#endif

#define MADERA_BASECLK_48K	49152000
#define MADERA_BASECLK_44K1	45158400

#define MADERA_AMP_RATE	48000
#define MADERA_AMP_BCLK	(MADERA_AMP_RATE * 16 * 2)

#define EXYNOS_PMU_PMU_DEBUG_OFFSET	0x0A00
#define MADERA_DAI_ID			0x4735
#define MADERA_CODEC_MAX		10
#define MADERA_AUX_MAX			2

#define RDMA_COUNT                      8
#define WDMA_COUNT                      5

#define UAIF_START			(RDMA_COUNT + WDMA_COUNT)
#define UAIF_COUNT			4
#define SIFS_START			(RDMA_COUNT + WDMA_COUNT + UAIF_COUNT + 2)
#define SIFS_COUNT			3

static unsigned int baserate = MADERA_BASECLK_48K;

enum FLL_ID { FLL1, FLL2, FLL3, FLLAO };
enum CLK_ID { SYSCLK, ASYNCCLK, DSPCLK, OPCLK, OUTCLK };

/* Used for debugging and test automation */
static u32 voice_trigger_count;

/* Debugfs value overrides, default to 0 */
static unsigned int forced_mclk1;
static unsigned int forced_sysclk;
static unsigned int forced_dspclk;

struct clk_conf {
	int id;
	const char *name;
	int source;
	int rate;
	int fout;

	bool valid;
};

#define MADERA_MAX_CLOCKS 10

struct madera_drvdata {
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

	int left_amp_dai;
	int right_amp_dai;
	struct clk *clk[MADERA_MAX_CLOCKS];
	unsigned int spk_id;
	int spk_id_value;
};

static struct madera_drvdata exynos9610_drvdata;
static struct snd_soc_card exynos9610_madera;

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

static struct snd_soc_pcm_runtime *madera_get_rtd(struct snd_soc_card *card,
		int id)
{
	struct snd_soc_dai_link *dai_link;
	struct snd_soc_pcm_runtime *rtd = NULL;

	for (dai_link = card->dai_link;
			dai_link - card->dai_link < card->num_links;
			dai_link++) {
		if (id == dai_link->id) {
			rtd = snd_soc_get_pcm_runtime(card, dai_link->name);
			break;
		}
	}

	if (!rtd)
		rtd = snd_soc_get_pcm_runtime(card, card->dai_link[id].name);

	return rtd;
}

static int madera_start_fll(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	unsigned int fsrc = 0, fin = 0, fout = 0, pll_id;
	int ret;

	if (!config->valid)
		return 0;

	codec_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;
	codec = codec_dai->codec;

	pll_id = map_fllid_with_name(config->name);
	switch (pll_id) {
	case FLL1:
		if (forced_mclk1) {
			/* use 32kHz input to avoid overclocking the FLL when
			 * forcing a specific MCLK frequency into the codec
			 * FLL calculations
			 */
#if IS_ENABLED(CONFIG_SND_SOC_MADERA)
			fsrc = MADERA_FLL_SRC_MCLK2;
#endif
			fin = forced_mclk1;
		} else {
			fsrc = config->source;
			fin = config->rate;
		}

		if (forced_sysclk)
			fout = forced_sysclk;
		else
			fout = config->fout;
		break;
	case FLL2:
	case FLLAO:
		fsrc = config->source;
		fin = config->rate;
		fout = config->fout;
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

static int madera_stop_fll(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	int ret;

	if (!config->valid)
		return 0;

	codec_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;
	codec = codec_dai->codec;

	ret = snd_soc_codec_set_pll(codec, config->id, 0, 0, 0);
	if (ret)
		dev_err(card->dev, "Failed to stop %s\n", config->name);

	return ret;
}

static int madera_set_clock(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *aif_dai;
	struct snd_soc_codec *codec;
	unsigned int freq = 0, clk_id;
	int ret;
	int dir = SND_SOC_CLOCK_IN;

	if (!config->valid)
		return 0;

	aif_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;
	codec = aif_dai->codec;

	clk_id = map_clkid_with_name(config->name);
	switch (clk_id) {
	case  SYSCLK:
		if (forced_sysclk)
			freq = forced_sysclk;
		else
			if (config->rate)
				freq = config->rate;
			else
				freq = baserate * 2;
		break;
	case ASYNCCLK:
		freq = config->rate;
		break;
	case DSPCLK:
		if (forced_dspclk)
			freq = forced_dspclk;
		else
			if (config->rate)
				freq = config->rate;
			else
				freq = baserate * 3;
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

static int madera_stop_clock(struct snd_soc_card *card,
				struct clk_conf *config)
{
	struct snd_soc_dai *aif_dai;
	struct snd_soc_codec *codec;
	int ret;

	if (!config->valid)
		return 0;

	aif_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;
	codec = aif_dai->codec;

	ret = snd_soc_codec_set_sysclk(codec, config->id, 0, 0, 0);
	if (ret)
		dev_err(card->dev, "Failed to stop %s\n", config->name);

	return ret;
}

static int madera_set_clocking(struct snd_soc_card *card,
				  struct madera_drvdata *drvdata)
{
	int ret;

	ret = madera_start_fll(card, &drvdata->fll1_refclk);
	if (ret)
		return ret;

	if (!drvdata->sysclk.rate) {
		ret = madera_set_clock(card, &drvdata->sysclk);
		if (ret)
			return ret;
	}

	if (!drvdata->dspclk.rate) {
		ret = madera_set_clock(card, &drvdata->dspclk);
		if (ret)
			return ret;
	}

	ret = madera_set_clock(card, &drvdata->opclk);
	if (ret)
		return ret;

	return ret;
}

static const struct snd_soc_ops rdma_ops = {
};

static const struct snd_soc_ops wdma_ops = {
};

static int madera_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct madera_drvdata *drvdata = card->drvdata;
	unsigned int rate = params_rate(params);
	int ret;

	/* Treat sysclk rate zero as automatic mode */
	if (!drvdata->sysclk.rate) {
		if (rate % 4000)
			baserate = MADERA_BASECLK_44K1;
		else
			baserate = MADERA_BASECLK_48K;
	}

	dev_dbg(card->dev, "Requesting Rate: %dHz, FLL: %dHz\n", rate,
		drvdata->sysclk.rate ? drvdata->sysclk.rate : baserate * 2);

	/* Ensure we can't race against set_bias_level */
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	ret = madera_set_clocking(card, drvdata);
	mutex_unlock(&card->dapm_mutex);

	return ret;
}

static int madera_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	return snd_soc_dai_set_tristate(rtd->cpu_dai, 0);
}

static void madera_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	snd_soc_dai_set_tristate(rtd->cpu_dai, 1);
}

static const struct snd_soc_ops uaif0_ops = {
	.hw_params = madera_hw_params,
	.prepare = madera_prepare,
	.shutdown = madera_shutdown,
};

static const struct snd_soc_ops uaif_ops = {
};

static int dsif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int tx_slot[] = {0, 1};

	/* bclk ratio 64 for DSD64, 128 for DSD128 */
	snd_soc_dai_set_bclk_ratio(cpu_dai, 64);

	/* channel map 0 1 if left is first, 1 0 if right is first */
	snd_soc_dai_set_channel_map(cpu_dai, 2, tx_slot, 0, NULL);
	return 0;
}

static const struct snd_soc_ops dsif_ops = {
	.hw_params = dsif_hw_params,
};

static int exynos9610_set_bias_level(struct snd_soc_card *card,
				  struct snd_soc_dapm_context *dapm,
				  enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai;
	struct madera_drvdata *drvdata = card->drvdata;
	int ret;

	codec_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_OFF)
			break;

		ret = madera_set_clocking(card, drvdata);
		if (ret)
			return ret;

		ret = madera_start_fll(card, &drvdata->fll2_refclk);
		if (ret)
			return ret;

		ret = madera_start_fll(card, &drvdata->fllao_refclk);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	return 0;
}

static int exynos9610_set_bias_level_post(struct snd_soc_card *card,
				       struct snd_soc_dapm_context *dapm,
				       enum snd_soc_bias_level level)
{
	struct snd_soc_dai *codec_dai;
	struct madera_drvdata *drvdata = card->drvdata;
	int ret;

	codec_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_OFF:
		ret = madera_stop_fll(card, &drvdata->fll1_refclk);
		if (ret)
			return ret;

		ret = madera_stop_fll(card, &drvdata->fll2_refclk);
		if (ret)
			return ret;

		ret = madera_stop_fll(card, &drvdata->fllao_refclk);
		if (ret)
			return ret;

		if (!drvdata->sysclk.rate) {
			ret = madera_stop_clock(card, &drvdata->sysclk);
			if (ret)
				return ret;
		}

		if (!drvdata->dspclk.rate) {
			ret = madera_stop_clock(card, &drvdata->dspclk);
			if (ret)
				return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_SND_SOC_MADERA)
static int madera_notify(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	const struct madera_hpdet_notify_data *hp_inf;
	const struct madera_micdet_notify_data *md_inf;
	const struct madera_voice_trigger_info *vt_inf;
	const struct madera_drvdata *drvdata =
		container_of(nb, struct madera_drvdata, nb);

	switch (event) {
	case MADERA_NOTIFY_VOICE_TRIGGER:
		vt_inf = data;
		dev_info(drvdata->dev, "Voice Triggered (core_num=%d)\n",
			 vt_inf->core_num);
		++voice_trigger_count;
		break;
	case MADERA_NOTIFY_HPDET:
		hp_inf = data;
		dev_info(drvdata->dev, "HPDET val=%d.%02d ohms\n",
			 hp_inf->impedance_x100 / 100,
			 hp_inf->impedance_x100 % 100);
		break;
	case MADERA_NOTIFY_MICDET:
		md_inf = data;
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
#endif

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int madera_force_fll1_enable_write(void *data, u64 val)
{
	struct snd_soc_card *card = data;
	struct madera_drvdata *drvdata = card->drvdata;
	int ret;

	if (val == 0)
		ret = madera_stop_fll(card, &drvdata->fll1_refclk);
	else
		ret = madera_start_fll(card, &drvdata->fll1_refclk);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(madera_force_fll1_enable_fops, NULL,
			madera_force_fll1_enable_write, "%llu\n");

static void madera_init_debugfs(struct snd_soc_card *card)
{
	struct dentry *root;

	if (!card->debugfs_card_root) {
		dev_warn(card->dev, "No card debugfs root\n");
		return;
	}

	root = debugfs_create_dir("test-automation", card->debugfs_card_root);
	if (!root) {
		dev_warn(card->dev, "Failed to create debugfs dir\n");
		return;
	}

	debugfs_create_u32("voice_trigger_count", S_IRUGO, root,
			   &voice_trigger_count);

	debugfs_create_u32("forced_mclk1", S_IRUGO | S_IWUGO, root,
			   &forced_mclk1);
	debugfs_create_u32("forced_sysclk", S_IRUGO | S_IWUGO, root,
			   &forced_sysclk);
	debugfs_create_u32("forced_dspclk", S_IRUGO | S_IWUGO, root,
			   &forced_dspclk);

	debugfs_create_file("force_fll1_enable", S_IWUSR | S_IWGRP, root, card,
			    &madera_force_fll1_enable_fops);
}
#else
static void madera_init_debugfs(struct snd_soc_card *card)
{
}
#endif

static int madera_amp_late_probe(struct snd_soc_card *card, int dai)
{
	struct madera_drvdata *drvdata = card->drvdata;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *amp_dai;
	struct snd_soc_codec *amp;
	int ret;
	dev_info(card->dev, "%s\n", __func__);

	if (!dai || !card->dai_link[dai].name)
		return 0;

	if (!drvdata->opclk.valid) {
		dev_err(card->dev, "OPCLK required to use speaker amp\n");
		return -ENOENT;
	}
	dev_info(card->dev, "%s: %s\n", __func__, card->dai_link[dai].name);

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[dai].name);
	amp_dai = rtd->codec_dai;
	amp = amp_dai->codec;

	/* using bclk for sysclk*/
	ret = snd_soc_codec_set_sysclk(amp, 0, 0, MADERA_AMP_BCLK,
				     SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(card->dev, "Failed to set amp DAI clock: %d\n", ret);
		return ret;
	}

	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(amp), "SPK AMP Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(amp), "SPK SPK");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(amp), "SPK VP");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(amp), "SPK VSENSE");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(amp), "SPK Main AMP");
	snd_soc_dapm_sync(snd_soc_codec_get_dapm(amp));

	return 0;
}

static int exynos9610_late_probe(struct snd_soc_card *card)
{
	struct madera_drvdata *drvdata = card->drvdata;
	struct snd_soc_dai *aif_dai;
	struct snd_soc_codec *codec;
	struct snd_soc_component *cpu;
	struct snd_soc_dapm_context *dapm;
	char name[SZ_32];
	const char *prefix;
	int ret, i;

	aif_dai = madera_get_rtd(card, 0)->cpu_dai;
	cpu = aif_dai->component;

	aif_dai = madera_get_rtd(card, MADERA_DAI_ID)->codec_dai;
	codec = aif_dai->codec;

	ret = snd_soc_dai_set_sysclk(aif_dai, drvdata->sysclk.id, 0, 0);
	if (ret != 0) {
		dev_err(drvdata->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	if (drvdata->sysclk.rate) {
		ret = madera_set_clock(card, &drvdata->sysclk);
		if (ret)
			return ret;
	}

	if (drvdata->dspclk.rate) {
		ret = madera_set_clock(card, &drvdata->dspclk);
		if (ret)
			return ret;
	}

	ret = madera_set_clock(card, &drvdata->asyncclk);
	if (ret)
		return ret;

	ret = madera_set_clock(card, &drvdata->outclk);
	if (ret)
		return ret;

	ret = madera_amp_late_probe(card, drvdata->left_amp_dai);
	if (ret)
		return ret;

	ret = madera_amp_late_probe(card, drvdata->right_amp_dai);
	if (ret)
		return ret;

	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUT");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUT1");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUT2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HEADSETMIC");
	snd_soc_dapm_ignore_suspend(&card->dapm, "RECEIVER");
	snd_soc_dapm_ignore_suspend(&card->dapm, "HEADPHONE");
	snd_soc_dapm_ignore_suspend(&card->dapm, "SPEAKER");
	snd_soc_dapm_ignore_suspend(&card->dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(&card->dapm, "DMIC2");
	snd_soc_dapm_ignore_suspend(&card->dapm, "DMIC3");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VTS Virtual Output");
	snd_soc_dapm_sync(&card->dapm);

	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF3 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS1");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS2");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICSUPP");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS1A");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS1B");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS2A");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "MICBIAS2B");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN1AL");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN1AR");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN1BL");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN1BR");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1TX1");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1TX2");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1RX1");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF1RX2");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2TX1");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2TX2");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2RX1");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2RX2");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN2AL");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN2R");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "DSP2 Virtual Output");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "DSP3 Virtual Output");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "DSP Virtual Input");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "DSP2 Trigger Out");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "DSP3 Trigger Out");
	snd_soc_dapm_sync(snd_soc_codec_get_dapm(codec));

	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA0 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA2 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA3 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA4 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA5 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA6 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX RDMA7 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA0 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA2 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA3 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(cpu), "ABOX WDMA4 Capture");
	snd_soc_dapm_sync(snd_soc_component_get_dapm(cpu));

	for (i = 0; i < UAIF_COUNT; i++) {
		aif_dai = madera_get_rtd(card, UAIF_START + i)->cpu_dai;
		cpu = aif_dai->component;
		dapm = snd_soc_component_get_dapm(cpu);
		prefix = dapm->component->name_prefix;
		snprintf(name, sizeof(name), "%s UAIF%d Capture", prefix, i);
		snd_soc_dapm_ignore_suspend(dapm, name);
		snprintf(name, sizeof(name), "%s UAIF%d Playback", prefix, i);
		snd_soc_dapm_ignore_suspend(dapm, name);
		snd_soc_dapm_sync(dapm);
	}

	for (i = 0; i < SIFS_COUNT; i++) {
		aif_dai = madera_get_rtd(card, SIFS_START + i)->cpu_dai;
		cpu = aif_dai->component;
		dapm = snd_soc_component_get_dapm(cpu);
		prefix = dapm->component->name_prefix;
		snprintf(name, sizeof(name), "%s SIFS%d Capture", prefix, i);
		snd_soc_dapm_ignore_suspend(dapm, name);
		snprintf(name, sizeof(name), "%s SIFS%d Playback", prefix, i);
		snd_soc_dapm_ignore_suspend(dapm, name);
		snd_soc_dapm_sync(dapm);
	}

	madera_init_debugfs(card);

#if IS_ENABLED(CONFIG_SND_SOC_MADERA)
	drvdata->nb.notifier_call = madera_notify;
	madera_register_notifier(codec, &drvdata->nb);
#endif

	return 0;
}

static struct snd_soc_pcm_stream madera_amp_params[] = {
	{
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = MADERA_AMP_RATE,
		.rate_max = MADERA_AMP_RATE,
		.channels_min = 1,
		.channels_max = 1,
	},
};

static struct snd_soc_dai_link exynos9610_dai[] = {
	{
		.name = "RDMA0",
		.stream_name = "RDMA0",
		.platform_name = "14a51000.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA1",
		.stream_name = "RDMA1",
		.platform_name = "14a51100.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA2",
		.stream_name = "RDMA2",
		.platform_name = "14a51200.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA3",
		.stream_name = "RDMA3",
		.platform_name = "14a51300.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA4",
		.stream_name = "RDMA4",
		.platform_name = "14a51400.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA5",
		.stream_name = "RDMA5",
		.platform_name = "14a51500.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA6",
		.stream_name = "RDMA6",
		.platform_name = "14a51600.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA7",
		.stream_name = "RDMA7",
		.platform_name = "14a51700.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "WDMA0",
		.stream_name = "WDMA0",
		.platform_name = "14a52000.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA1",
		.stream_name = "WDMA1",
		.platform_name = "14a52100.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA2",
		.stream_name = "WDMA2",
		.platform_name = "14a52200.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA3",
		.stream_name = "WDMA3",
		.platform_name = "14a52300.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA4",
		.stream_name = "WDMA4",
		.platform_name = "14a52400.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
#if IS_ENABLED(SND_SOC_SAMSUNG_DISPLAYPORT)
	{
		.name = "DP Audio",
		.stream_name = "DP Audio",
		.cpu_dai_name = "audio_cpu_dummy",
		.platform_name = "dp_dma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
	},
#endif
	{
		.name = "UAIF0",
		.stream_name = "UAIF0",
		.platform_name = "snd-soc-dummy",
		.id = MADERA_DAI_ID,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif0_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF1",
		.stream_name = "UAIF1",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF2",
		.stream_name = "UAIF2",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF4",
		.stream_name = "UAIF4",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "DSIF",
		.stream_name = "DSIF",
		.cpu_dai_name = "DSIF",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &dsif_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "SPDY",
		.stream_name = "SPDY",
		.cpu_dai_name = "SPDY",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_capture = 1,
	},
	{
		.name = "SIFS0",
		.stream_name = "SIFS0",
		.cpu_dai_name = "SIFS0",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SIFS1",
		.stream_name = "SIFS1",
		.cpu_dai_name = "SIFS1",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SIFS2",
		.stream_name = "SIFS2",
		.cpu_dai_name = "SIFS2",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "codec-left-amp",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.params = madera_amp_params,
	},

	{
		.name = "cpu-dsp-voice-control",
		.stream_name = "cpu-dsp-voice-control",
		.cpu_dai_name = "cs47l35-cpu-voicectrl",
		.platform_name = "cs47l35-codec",
		.codec_dai_name = "cs47l35-dsp-voicectrl",
		.codec_name = "cs47l35-codec",
		.ignore_suspend = 1,
		.dynamic = 0,
	},
	{
		.name = "cpu-dsp-trace",
		.stream_name = "cpu-dsp-voice-trace",
		.cpu_dai_name = "cs47l35-cpu-trace",
		.platform_name = "cs47l35-codec",
		.codec_dai_name = "cs47l35-dsp-trace",
		.codec_name = "cs47l35-codec",
		.ignore_suspend = 1,
		.dynamic = 0,
	},
	{
		.name = "cpu-dsp2-text",
		.stream_name = "cpu-dsp2-text",
		.cpu_dai_name = "cs47l35-dsp2-cpu-txt",
		.platform_name = "cs47l35-codec",
		.codec_dai_name = "cs47l35-dsp2-txt",
		.codec_name = "cs47l35-codec",
		.ignore_suspend = 1,
		.dynamic = 0,
	},
	{
		.name = "cpu-dsp3-text",
		.stream_name = "cpu-dsp3-text",
		.cpu_dai_name = "cs47l35-dsp3-cpu-txt",
		.platform_name = "cs47l35-codec",
		.codec_dai_name = "cs47l35-dsp3-txt",
		.codec_name = "cs47l35-codec",
		.ignore_suspend = 1,
		.dynamic = 0,
	},
	{
		.name = "cpu-dsp1-text",
		.stream_name = "cpu-dsp1-text",
		.cpu_dai_name = "cs47l35-dsp1-cpu-txt",
		.platform_name = "cs47l35-codec",
		.codec_dai_name = "cs47l35-dsp1-txt",
		.codec_name = "cs47l35-codec",
		.ignore_suspend = 1,
		.dynamic = 0,
	}

};

static const char * const vts_output_texts[] = {
        "None",
        "DMIC1",
};

static int madera_spk_id_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = &exynos9610_madera;
	struct madera_drvdata *drvdata = card->drvdata;

	dev_info(drvdata->dev, "%s(%d)\n", __func__, drvdata->spk_id_value);
	ucontrol->value.integer.value[0] = drvdata->spk_id_value;

	return 0;
}

static const struct snd_kcontrol_new exynos9610_controls[] = {
	SOC_DAPM_PIN_SWITCH("DMIC1"),
	SOC_DAPM_PIN_SWITCH("DMIC2"),
	SOC_DAPM_PIN_SWITCH("DMIC3"),
	SOC_SINGLE_EXT("SPK ID", SND_SOC_NOPM, 0, 1, 0,
			madera_spk_id_get, NULL),
};

static struct snd_soc_dapm_widget exynos9610_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTPUT"),
	SND_SOC_DAPM_INPUT("VINPUT1"),
	SND_SOC_DAPM_INPUT("VINPUT2"),
	SND_SOC_DAPM_OUTPUT("VOUTPUTCALL"),
	SND_SOC_DAPM_INPUT("VINPUTCALL"),
	SND_SOC_DAPM_MIC("DMIC1", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_MIC("DMIC3", NULL),
	SND_SOC_DAPM_MIC("HEADSETMIC", NULL),
	SND_SOC_DAPM_SPK("RECEIVER", NULL),
	SND_SOC_DAPM_HP("HEADPHONE", NULL),
	SND_SOC_DAPM_SPK("SPEAKER", NULL),
	SND_SOC_DAPM_MIC("BLUETOOTH MIC", NULL),
	SND_SOC_DAPM_SPK("BLUETOOTH SPK", NULL),
};

static struct snd_soc_codec_conf codec_conf[MADERA_CODEC_MAX];

static struct snd_soc_aux_dev aux_dev[MADERA_AUX_MAX];

static struct snd_soc_card exynos9610_madera = {
	.name = "Exynos9610-Madera",
	.owner = THIS_MODULE,
	.dai_link = exynos9610_dai,
	.num_links = ARRAY_SIZE(exynos9610_dai),

	.late_probe = exynos9610_late_probe,

	.controls = exynos9610_controls,
	.num_controls = ARRAY_SIZE(exynos9610_controls),
	.dapm_widgets = exynos9610_widgets,
	.num_dapm_widgets = ARRAY_SIZE(exynos9610_widgets),

	.set_bias_level = exynos9610_set_bias_level,
	.set_bias_level_post = exynos9610_set_bias_level_post,

	.drvdata = (void *)&exynos9610_drvdata,

	.codec_conf = codec_conf,
	.num_configs = ARRAY_SIZE(codec_conf),

	.aux_dev = aux_dev,
	.num_aux_devs = ARRAY_SIZE(aux_dev),
};

static int read_clk_conf(struct device_node *np,
				   const char * const prop,
				   struct clk_conf *conf,
				   bool is_fll)
{
	u32 tmp;
	int ret;

	/*Truncate "cirrus," from prop_name to fetch clk_name*/
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

	if (is_fll) {
		ret = of_property_read_u32_index(np, prop, 3, &tmp);
		if (ret)
			return ret;
		conf->fout = tmp;
	}

	conf->valid = true;

	return 0;
}

static int read_dai(struct device_node *np, const char * const prop,
			      struct device_node **dai, const char **name)
{
	int ret = 0;

	np = of_get_child_by_name(np, prop);
	if (!np)
		return -ENOENT;

	*dai = of_parse_phandle(np, "sound-dai", 0);
	if (!*dai) {
		ret = -ENODEV;
		goto out;
	}

	if (*name == NULL) {
		/* Ignoring the return as we don't register DAIs to the platform */
		ret = snd_soc_of_get_dai_name(np, name);
		if (ret && !*name)
			return ret;
	}
out:
	of_node_put(np);

	return ret;
}

static struct clk *xclkout;

static void control_xclkout(bool on)
{
	if (on) {
		clk_prepare_enable(xclkout);
	} else {
		clk_disable_unprepare(xclkout);
	}
}

static int exynos9610_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &exynos9610_madera;
	struct madera_drvdata *drvdata = card->drvdata;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dai;
	int nlink = 0;
	int i, rc, ret;
	const char *cur = NULL;
	struct property *p;

	card->dev = &pdev->dev;
	drvdata->dev = card->dev;

	snd_soc_card_set_drvdata(card, drvdata);

	xclkout = devm_clk_get(&pdev->dev, "xclkout");
	if (IS_ERR(xclkout)) {
		dev_err(&pdev->dev, "xclkout get failed\n");
		xclkout = NULL;
	}
	control_xclkout(true);
	dev_info(&pdev->dev, "xclkout is enabled\n");

	i = 0;
	p = of_find_property(np, "clock-names", NULL);
	if (p) {
		while ((cur = of_prop_next_string(p, cur)) != NULL) {
			drvdata->clk[i] = devm_clk_get(drvdata->dev, cur);
			if (IS_ERR(drvdata->clk[i])) {
				dev_info(drvdata->dev, "Failed to get %s: %ld\n",
					 cur, PTR_ERR(drvdata->clk[i]));
				drvdata->clk[i] = NULL;
				break;
			}

			clk_prepare_enable(drvdata->clk[i]);

			if (++i == MADERA_MAX_CLOCKS)
				break;
		}
	}

	ret = read_clk_conf(np, "cirrus,sysclk",
			&drvdata->sysclk, false);
	if (ret) {
		dev_err(card->dev, "Failed to parse sysclk: %d\n", ret);
		return ret;
	}
	ret = read_clk_conf(np, "cirrus,asyncclk",
			&drvdata->asyncclk, false);
	if (ret)
		dev_info(card->dev, "Failed to parse asyncclk: %d\n", ret);

	ret = read_clk_conf(np, "cirrus,dspclk",
			&drvdata->dspclk, false);
	if (ret) {
		dev_info(card->dev, "Failed to parse dspclk: %d\n", ret);
	}

	ret = read_clk_conf(np, "cirrus,opclk",
			&drvdata->opclk, false);
	if (ret)
		dev_info(card->dev, "Failed to parse opclk: %d\n", ret);

	ret = read_clk_conf(np, "cirrus,fll1-refclk",
			&drvdata->fll1_refclk, true);
	if (ret)
		dev_info(card->dev, "Failed to parse fll1-refclk: %d\n", ret);

	ret = read_clk_conf(np, "cirrus,fll2-refclk",
			&drvdata->fll2_refclk, true);
	if (ret)
		dev_info(card->dev, "Failed to parse fll2-refclk: %d\n", ret);

	ret = read_clk_conf(np, "cirrus,fllao-refclk",
			&drvdata->fllao_refclk, true);
	if (ret)
		dev_info(card->dev, "Failed to parse fllao-refclk: %d\n", ret);

	ret = read_clk_conf(np, "cirrus,outclk",
			&drvdata->outclk, false);
	if (ret)
		dev_info(card->dev, "Failed to parse outclk: %d\n", ret);

	if (of_get_property(np, "gpios", NULL) != NULL)  {
		drvdata->spk_id = of_get_gpio(np, 0);
		if (drvdata->spk_id < 0) {
			dev_err(&pdev->dev, "failed to get SPK ID GPIO");
		} else {
			drvdata->spk_id_value = gpio_get_value(drvdata->spk_id);
			dev_info(&pdev->dev, "SPK_ID gpio value = %d\n", drvdata->spk_id_value);
		}
	}

	for_each_child_of_node(np, dai) {
		if (!exynos9610_dai[nlink].name)
			exynos9610_dai[nlink].name = dai->name;
		if (!exynos9610_dai[nlink].stream_name)
			exynos9610_dai[nlink].stream_name = dai->name;

		if (!exynos9610_dai[nlink].cpu_name) {
			ret = read_dai(dai, "cpu",
					&exynos9610_dai[nlink].cpu_of_node,
					&exynos9610_dai[nlink].cpu_dai_name);
			if (ret) {
				dev_err(card->dev,
					"Failed to parse cpu DAI for %s: %d\n",
					dai->name, ret);
				return ret;
			}
		}

		if (!exynos9610_dai[nlink].platform_name) {
			ret = read_dai(dai, "platform",
				&exynos9610_dai[nlink].platform_of_node,
				&exynos9610_dai[nlink].platform_name);
			if (ret) {
				exynos9610_dai[nlink].platform_of_node =
					exynos9610_dai[nlink].cpu_of_node;
				dev_info(card->dev,
					"Cpu node is used as platform for %s: %d\n",
					dai->name, ret);
			}
		}

		if (!exynos9610_dai[nlink].codec_name) {
			ret = read_dai(dai, "codec",
					&exynos9610_dai[nlink].codec_of_node,
					&exynos9610_dai[nlink].codec_dai_name);
			if (ret) {
				dev_err(card->dev,
					"Failed to parse codec DAI for %s: %d\n",
					dai->name, ret);
				return ret;
			}
		}

		if (strstr(dai->name, "left-amp")) {
			exynos9610_dai[nlink].params = madera_amp_params;
			drvdata->left_amp_dai = nlink;
		} else if (strstr(dai->name, "right-amp")) {
			exynos9610_dai[nlink].params = madera_amp_params;
			drvdata->right_amp_dai = nlink;
		}

		exynos9610_dai[nlink].dai_fmt =
				snd_soc_of_parse_daifmt(dai, NULL, NULL, NULL);

		if (++nlink == card->num_links)
			break;
	}

	if (!nlink) {
		dev_err(card->dev, "No DAIs specified\n");
		return -EINVAL;
	}

	if (of_property_read_bool(np, "samsung,routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "samsung,routing");
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(codec_conf); i++) {
		codec_conf[i].of_node = of_parse_phandle(np, "samsung,codec", i);
		if (IS_ERR_OR_NULL(codec_conf[i].of_node)) {
			exynos9610_madera.num_configs = i;
			break;
		}

		rc = of_property_read_string_index(np, "samsung,prefix", i,
				&codec_conf[i].name_prefix);
		if (rc < 0)
			codec_conf[i].name_prefix = "";
	}

	for (i = 0; i < ARRAY_SIZE(aux_dev); i++) {
		aux_dev[i].codec_of_node = of_parse_phandle(np, "samsung,aux", i);
		if (IS_ERR_OR_NULL(aux_dev[i].codec_of_node)) {
			exynos9610_madera.num_aux_devs = i;
			break;
		}
	}

	ret = devm_snd_soc_register_card(card->dev, card);
	if (ret)
		dev_err(card->dev, "snd_soc_register_card() failed:%d\n", ret);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id exynos9610_of_match[] = {
	{ .compatible = "samsung,exynos9610-madera", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos9610_of_match);
#endif /* CONFIG_OF */

static struct platform_driver exynos9610_audio_driver = {
	.driver		= {
		.name	= "exynos9610-madera",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(exynos9610_of_match),
	},

	.probe		= exynos9610_audio_probe,
};

module_platform_driver(exynos9610_audio_driver);

MODULE_DESCRIPTION("ALSA SoC Exynos9610 Madera Driver");
MODULE_AUTHOR("Charles Keepax <ckeepax@opensource.wolfsonmicro.com>");
MODULE_AUTHOR("Gyeongtaek Lee <gt82.lee@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos9610-madera");
