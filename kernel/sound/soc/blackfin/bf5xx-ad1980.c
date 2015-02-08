

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <asm/dma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <linux/gpio.h>
#include <asm/portmux.h>

#include "../codecs/ad1980.h"
#include "bf5xx-sport.h"
#include "bf5xx-ac97-pcm.h"
#include "bf5xx-ac97.h"

static struct snd_soc_card bf5xx_board;

static int bf5xx_board_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	pr_debug("%s enter\n", __func__);
	cpu_dai->private_data = sport_handle;
	return 0;
}

static struct snd_soc_ops bf5xx_board_ops = {
	.startup = bf5xx_board_startup,
};

static struct snd_soc_dai_link bf5xx_board_dai = {
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &bfin_ac97_dai,
	.codec_dai = &ad1980_dai,
	.ops = &bf5xx_board_ops,
};

static struct snd_soc_card bf5xx_board = {
	.name = "bf5xx-board",
	.platform = &bf5xx_ac97_soc_platform,
	.dai_link = &bf5xx_board_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_board_snd_devdata = {
	.card = &bf5xx_board,
	.codec_dev = &soc_codec_dev_ad1980,
};

static struct platform_device *bf5xx_board_snd_device;

static int __init bf5xx_board_init(void)
{
	int ret;

	bf5xx_board_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_board_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_board_snd_device, &bf5xx_board_snd_devdata);
	bf5xx_board_snd_devdata.dev = &bf5xx_board_snd_device->dev;
	ret = platform_device_add(bf5xx_board_snd_device);

	if (ret)
		platform_device_put(bf5xx_board_snd_device);

	return ret;
}

static void __exit bf5xx_board_exit(void)
{
	platform_device_unregister(bf5xx_board_snd_device);
}

module_init(bf5xx_board_init);
module_exit(bf5xx_board_exit);

/* Module information */
MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("ALSA SoC AD1980/1 BF5xx board");
MODULE_LICENSE("GPL");
