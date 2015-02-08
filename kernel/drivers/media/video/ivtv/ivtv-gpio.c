

#include "ivtv-driver.h"
#include "ivtv-cards.h"
#include "ivtv-gpio.h"
#include "tuner-xc2028.h"
#include <media/tuner.h>


/********************* GPIO stuffs *********************/

/* GPIO registers */
#define IVTV_REG_GPIO_IN    0x9008
#define IVTV_REG_GPIO_OUT   0x900c
#define IVTV_REG_GPIO_DIR   0x9020

void ivtv_reset_ir_gpio(struct ivtv *itv)
{
	int curdir, curout;

	if (itv->card->type != IVTV_CARD_PVR_150)
		return;
	IVTV_DEBUG_INFO("Resetting PVR150 IR\n");
	curout = read_reg(IVTV_REG_GPIO_OUT);
	curdir = read_reg(IVTV_REG_GPIO_DIR);
	curdir |= 0x80;
	write_reg(curdir, IVTV_REG_GPIO_DIR);
	curout = (curout & ~0xF) | 1;
	write_reg(curout, IVTV_REG_GPIO_OUT);
	/* We could use something else for smaller time */
	schedule_timeout_interruptible(msecs_to_jiffies(1));
	curout |= 2;
	write_reg(curout, IVTV_REG_GPIO_OUT);
	curdir &= ~0x80;
	write_reg(curdir, IVTV_REG_GPIO_DIR);
}

/* Xceive tuner reset function */
int ivtv_reset_tuner_gpio(void *dev, int component, int cmd, int value)
{
	struct i2c_algo_bit_data *algo = dev;
	struct ivtv *itv = algo->data;
	u32 curout;

	if (cmd != XC2028_TUNER_RESET)
		return 0;
	IVTV_DEBUG_INFO("Resetting tuner\n");
	curout = read_reg(IVTV_REG_GPIO_OUT);
	curout &= ~(1 << itv->card->xceive_pin);
	write_reg(curout, IVTV_REG_GPIO_OUT);
	schedule_timeout_interruptible(msecs_to_jiffies(1));

	curout |= 1 << itv->card->xceive_pin;
	write_reg(curout, IVTV_REG_GPIO_OUT);
	schedule_timeout_interruptible(msecs_to_jiffies(1));
	return 0;
}

static inline struct ivtv *sd_to_ivtv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ivtv, sd_gpio);
}

static struct v4l2_queryctrl gpio_ctrl_mute = {
	.id            = V4L2_CID_AUDIO_MUTE,
	.type          = V4L2_CTRL_TYPE_BOOLEAN,
	.name          = "Mute",
	.minimum       = 0,
	.maximum       = 1,
	.step          = 1,
	.default_value = 1,
	.flags         = 0,
};

static int subdev_s_clock_freq(struct v4l2_subdev *sd, u32 freq)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	mask = itv->card->gpio_audio_freq.mask;
	switch (freq) {
	case 32000:
		data = itv->card->gpio_audio_freq.f32000;
		break;
	case 44100:
		data = itv->card->gpio_audio_freq.f44100;
		break;
	case 48000:
	default:
		data = itv->card->gpio_audio_freq.f48000;
		break;
	}
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static int subdev_g_tuner(struct v4l2_subdev *sd, struct v4l2_tuner *vt)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask;

	mask = itv->card->gpio_audio_detect.mask;
	if (mask == 0 || (read_reg(IVTV_REG_GPIO_IN) & mask))
		vt->rxsubchans = V4L2_TUNER_SUB_STEREO |
			V4L2_TUNER_SUB_LANG1 | V4L2_TUNER_SUB_LANG2;
	else
		vt->rxsubchans = V4L2_TUNER_SUB_MONO;
	return 0;
}

static int subdev_s_tuner(struct v4l2_subdev *sd, struct v4l2_tuner *vt)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	mask = itv->card->gpio_audio_mode.mask;
	switch (vt->audmode) {
	case V4L2_TUNER_MODE_LANG1:
		data = itv->card->gpio_audio_mode.lang1;
		break;
	case V4L2_TUNER_MODE_LANG2:
		data = itv->card->gpio_audio_mode.lang2;
		break;
	case V4L2_TUNER_MODE_MONO:
		data = itv->card->gpio_audio_mode.mono;
		break;
	case V4L2_TUNER_MODE_STEREO:
	case V4L2_TUNER_MODE_LANG1_LANG2:
	default:
		data = itv->card->gpio_audio_mode.stereo;
		break;
	}
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static int subdev_s_radio(struct v4l2_subdev *sd)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	mask = itv->card->gpio_audio_input.mask;
	data = itv->card->gpio_audio_input.radio;
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static int subdev_s_audio_routing(struct v4l2_subdev *sd,
				  u32 input, u32 output, u32 config)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	if (input > 2)
		return -EINVAL;
	mask = itv->card->gpio_audio_input.mask;
	switch (input) {
	case 0:
		data = itv->card->gpio_audio_input.tuner;
		break;
	case 1:
		data = itv->card->gpio_audio_input.linein;
		break;
	case 2:
	default:
		data = itv->card->gpio_audio_input.radio;
		break;
	}
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static int subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	if (ctrl->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	mask = itv->card->gpio_audio_mute.mask;
	data = itv->card->gpio_audio_mute.mute;
	ctrl->value = (read_reg(IVTV_REG_GPIO_OUT) & mask) == data;
	return 0;
}

static int subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	if (ctrl->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	mask = itv->card->gpio_audio_mute.mask;
	data = ctrl->value ? itv->card->gpio_audio_mute.mute : 0;
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static int subdev_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	if (qc->id != V4L2_CID_AUDIO_MUTE)
		return -EINVAL;
	*qc = gpio_ctrl_mute;
	return 0;
}

static int subdev_log_status(struct v4l2_subdev *sd)
{
	struct ivtv *itv = sd_to_ivtv(sd);

	IVTV_INFO("GPIO status: DIR=0x%04x OUT=0x%04x IN=0x%04x\n",
			read_reg(IVTV_REG_GPIO_DIR), read_reg(IVTV_REG_GPIO_OUT),
			read_reg(IVTV_REG_GPIO_IN));
	return 0;
}

static int subdev_s_video_routing(struct v4l2_subdev *sd,
				  u32 input, u32 output, u32 config)
{
	struct ivtv *itv = sd_to_ivtv(sd);
	u16 mask, data;

	if (input > 2) /* 0:Tuner 1:Composite 2:S-Video */
		return -EINVAL;
	mask = itv->card->gpio_video_input.mask;
	if (input == 0)
		data = itv->card->gpio_video_input.tuner;
	else if (input == 1)
		data = itv->card->gpio_video_input.composite;
	else
		data = itv->card->gpio_video_input.svideo;
	if (mask)
		write_reg((read_reg(IVTV_REG_GPIO_OUT) & ~mask) | (data & mask), IVTV_REG_GPIO_OUT);
	return 0;
}

static const struct v4l2_subdev_core_ops subdev_core_ops = {
	.log_status = subdev_log_status,
	.g_ctrl = subdev_g_ctrl,
	.s_ctrl = subdev_s_ctrl,
	.queryctrl = subdev_queryctrl,
};

static const struct v4l2_subdev_tuner_ops subdev_tuner_ops = {
	.s_radio = subdev_s_radio,
	.g_tuner = subdev_g_tuner,
	.s_tuner = subdev_s_tuner,
};

static const struct v4l2_subdev_audio_ops subdev_audio_ops = {
	.s_clock_freq = subdev_s_clock_freq,
	.s_routing = subdev_s_audio_routing,
};

static const struct v4l2_subdev_video_ops subdev_video_ops = {
	.s_routing = subdev_s_video_routing,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &subdev_core_ops,
	.tuner = &subdev_tuner_ops,
	.audio = &subdev_audio_ops,
	.video = &subdev_video_ops,
};

int ivtv_gpio_init(struct ivtv *itv)
{
	u16 pin = 0;

	if (itv->card->xceive_pin)
		pin = 1 << itv->card->xceive_pin;

	if ((itv->card->gpio_init.direction | pin) == 0)
		return 0;

	IVTV_DEBUG_INFO("GPIO initial dir: %08x out: %08x\n",
		   read_reg(IVTV_REG_GPIO_DIR), read_reg(IVTV_REG_GPIO_OUT));

	/* init output data then direction */
	write_reg(itv->card->gpio_init.initial_value | pin, IVTV_REG_GPIO_OUT);
	write_reg(itv->card->gpio_init.direction | pin, IVTV_REG_GPIO_DIR);
	v4l2_subdev_init(&itv->sd_gpio, &subdev_ops);
	snprintf(itv->sd_gpio.name, sizeof(itv->sd_gpio.name), "%s-gpio", itv->v4l2_dev.name);
	itv->sd_gpio.grp_id = IVTV_HW_GPIO;
	return v4l2_device_register_subdev(&itv->v4l2_dev, &itv->sd_gpio);
}
