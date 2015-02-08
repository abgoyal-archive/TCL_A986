

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/tvp514x.h>

#include "tvp514x_regs.h"

/* Module Name */
#define TVP514X_MODULE_NAME		"tvp514x"

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TVP514X linux decoder driver");
MODULE_LICENSE("GPL");

/* enum tvp514x_std - enum for supported standards */
enum tvp514x_std {
	STD_NTSC_MJ = 0,
	STD_PAL_BDGHIN,
	STD_INVALID
};

struct tvp514x_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

static struct tvp514x_reg tvp514x_reg_list_default[0x40];

static int tvp514x_s_stream(struct v4l2_subdev *sd, int enable);
struct tvp514x_decoder {
	struct v4l2_subdev sd;
	struct tvp514x_reg tvp514x_regs[ARRAY_SIZE(tvp514x_reg_list_default)];
	const struct tvp514x_platform_data *pdata;

	int ver;
	int streaming;

	enum tvp514x_std current_std;
	int num_stds;
	const struct tvp514x_std_info *std_list;
	/* Input and Output Routing parameters */
	u32 input;
	u32 output;
};

/* TVP514x default register values */
static struct tvp514x_reg tvp514x_reg_list_default[] = {
	/* Composite selected */
	{TOK_WRITE, REG_INPUT_SEL, 0x05},
	{TOK_WRITE, REG_AFE_GAIN_CTRL, 0x0F},
	/* Auto mode */
	{TOK_WRITE, REG_VIDEO_STD, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_SKIP, REG_AUTOSWITCH_MASK, 0x3F},
	{TOK_WRITE, REG_COLOR_KILLER, 0x10},
	{TOK_WRITE, REG_LUMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL2, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL3, 0x02},
	{TOK_WRITE, REG_BRIGHTNESS, 0x80},
	{TOK_WRITE, REG_CONTRAST, 0x80},
	{TOK_WRITE, REG_SATURATION, 0x80},
	{TOK_WRITE, REG_HUE, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL2, 0x0E},
	/* Reserved */
	{TOK_SKIP, 0x0F, 0x00},
	{TOK_WRITE, REG_COMP_PR_SATURATION, 0x80},
	{TOK_WRITE, REG_COMP_Y_CONTRAST, 0x80},
	{TOK_WRITE, REG_COMP_PB_SATURATION, 0x80},
	/* Reserved */
	{TOK_SKIP, 0x13, 0x00},
	{TOK_WRITE, REG_COMP_Y_BRIGHTNESS, 0x80},
	/* Reserved */
	{TOK_SKIP, 0x15, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_AVID_START_PIXEL_LSB, 0x55},
	{TOK_SKIP, REG_AVID_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_LSB, 0x25},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_MSB, 0x03},
	/* NTSC timing */
	{TOK_SKIP, REG_HSYNC_START_PIXEL_LSB, 0x00},
	{TOK_SKIP, REG_HSYNC_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_LSB, 0x40},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_MSB, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_VSYNC_START_LINE_LSB, 0x04},
	{TOK_SKIP, REG_VSYNC_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_LSB, 0x07},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_MSB, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_VBLK_START_LINE_LSB, 0x01},
	{TOK_SKIP, REG_VBLK_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_STOP_LINE_LSB, 0x15},
	{TOK_SKIP, REG_VBLK_STOP_LINE_MSB, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x26, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x27, 0x00},
	{TOK_SKIP, REG_FAST_SWTICH_CONTROL, 0xCC},
	/* Reserved */
	{TOK_SKIP, 0x29, 0x00},
	{TOK_SKIP, REG_FAST_SWTICH_SCART_DELAY, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x2B, 0x00},
	{TOK_SKIP, REG_SCART_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_CONTROL, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x2F, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x30, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x31, 0x00},
	/* HS, VS active high */
	{TOK_WRITE, REG_SYNC_CONTROL, 0x00},
	/* 10-bit BT.656 */
	{TOK_WRITE, REG_OUTPUT_FORMATTER1, 0x00},
	/* Enable clk & data */
	{TOK_WRITE, REG_OUTPUT_FORMATTER2, 0x11},
	/* Enable AVID & FLD */
	{TOK_WRITE, REG_OUTPUT_FORMATTER3, 0xEE},
	/* Enable VS & HS */
	{TOK_WRITE, REG_OUTPUT_FORMATTER4, 0xAF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER5, 0xFF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER6, 0xFF},
	/* Clear status */
	{TOK_WRITE, REG_CLEAR_LOST_LOCK, 0x01},
	{TOK_TERM, 0, 0},
};

static const struct tvp514x_std_info tvp514x_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	[STD_NTSC_MJ] = {
	 .width = NTSC_NUM_ACTIVE_PIXELS,
	 .height = NTSC_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_NTSC_MJ_BIT,
	 .standard = {
		      .index = 0,
		      .id = V4L2_STD_NTSC,
		      .name = "NTSC",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	/* Standard: STD_PAL_BDGHIN */
	},
	[STD_PAL_BDGHIN] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_BDGHIN_BIT,
	 .standard = {
		      .index = 1,
		      .id = V4L2_STD_PAL,
		      .name = "PAL",
		      .frameperiod = {1, 25},
		      .framelines = 625
		     },
	},
	/* Standard: need to add for additional standard */
};


static inline struct tvp514x_decoder *to_decoder(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tvp514x_decoder, sd);
}


static int tvp514x_read_reg(struct v4l2_subdev *sd, u8 reg)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

read_again:

	err = i2c_smbus_read_byte_data(client, reg);
	if (err < 0) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Read: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto read_again;
		}
	}

	return err;
}

static void dump_reg(struct v4l2_subdev *sd, u8 reg)
{
	u32 val;

	val = tvp514x_read_reg(sd, reg);
	v4l2_info(sd, "Reg(0x%.2X): 0x%.2X\n", reg, val);
}

static int tvp514x_write_reg(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

write_again:

	err = i2c_smbus_write_byte_data(client, reg, val);
	if (err) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Write: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto write_again;
		}
	}

	return err;
}

static int tvp514x_write_regs(struct v4l2_subdev *sd,
			      const struct tvp514x_reg reglist[])
{
	int err;
	const struct tvp514x_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp514x_write_reg(sd, next->reg, (u8) next->val);
		if (err) {
			v4l2_err(sd, "Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}

static enum tvp514x_std tvp514x_query_current_std(struct v4l2_subdev *sd)
{
	u8 std, std_status;

	std = tvp514x_read_reg(sd, REG_VIDEO_STD);
	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT)
		/* use the standard status register */
		std_status = tvp514x_read_reg(sd, REG_VIDEO_STD_STATUS);
	else
		/* use the standard register itself */
		std_status = std;

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
		return STD_NTSC_MJ;

	case VIDEO_STD_PAL_BDGHIN_BIT:
		return STD_PAL_BDGHIN;

	default:
		return STD_INVALID;
	}

	return STD_INVALID;
}

/* TVP5146/47 register dump function */
static void tvp514x_reg_dump(struct v4l2_subdev *sd)
{
	dump_reg(sd, REG_INPUT_SEL);
	dump_reg(sd, REG_AFE_GAIN_CTRL);
	dump_reg(sd, REG_VIDEO_STD);
	dump_reg(sd, REG_OPERATION_MODE);
	dump_reg(sd, REG_COLOR_KILLER);
	dump_reg(sd, REG_LUMA_CONTROL1);
	dump_reg(sd, REG_LUMA_CONTROL2);
	dump_reg(sd, REG_LUMA_CONTROL3);
	dump_reg(sd, REG_BRIGHTNESS);
	dump_reg(sd, REG_CONTRAST);
	dump_reg(sd, REG_SATURATION);
	dump_reg(sd, REG_HUE);
	dump_reg(sd, REG_CHROMA_CONTROL1);
	dump_reg(sd, REG_CHROMA_CONTROL2);
	dump_reg(sd, REG_COMP_PR_SATURATION);
	dump_reg(sd, REG_COMP_Y_CONTRAST);
	dump_reg(sd, REG_COMP_PB_SATURATION);
	dump_reg(sd, REG_COMP_Y_BRIGHTNESS);
	dump_reg(sd, REG_AVID_START_PIXEL_LSB);
	dump_reg(sd, REG_AVID_START_PIXEL_MSB);
	dump_reg(sd, REG_AVID_STOP_PIXEL_LSB);
	dump_reg(sd, REG_AVID_STOP_PIXEL_MSB);
	dump_reg(sd, REG_HSYNC_START_PIXEL_LSB);
	dump_reg(sd, REG_HSYNC_START_PIXEL_MSB);
	dump_reg(sd, REG_HSYNC_STOP_PIXEL_LSB);
	dump_reg(sd, REG_HSYNC_STOP_PIXEL_MSB);
	dump_reg(sd, REG_VSYNC_START_LINE_LSB);
	dump_reg(sd, REG_VSYNC_START_LINE_MSB);
	dump_reg(sd, REG_VSYNC_STOP_LINE_LSB);
	dump_reg(sd, REG_VSYNC_STOP_LINE_MSB);
	dump_reg(sd, REG_VBLK_START_LINE_LSB);
	dump_reg(sd, REG_VBLK_START_LINE_MSB);
	dump_reg(sd, REG_VBLK_STOP_LINE_LSB);
	dump_reg(sd, REG_VBLK_STOP_LINE_MSB);
	dump_reg(sd, REG_SYNC_CONTROL);
	dump_reg(sd, REG_OUTPUT_FORMATTER1);
	dump_reg(sd, REG_OUTPUT_FORMATTER2);
	dump_reg(sd, REG_OUTPUT_FORMATTER3);
	dump_reg(sd, REG_OUTPUT_FORMATTER4);
	dump_reg(sd, REG_OUTPUT_FORMATTER5);
	dump_reg(sd, REG_OUTPUT_FORMATTER6);
	dump_reg(sd, REG_CLEAR_LOST_LOCK);
}

static int tvp514x_configure(struct v4l2_subdev *sd,
		struct tvp514x_decoder *decoder)
{
	int err;

	/* common register initialization */
	err =
	    tvp514x_write_regs(sd, decoder->tvp514x_regs);
	if (err)
		return err;

	if (debug)
		tvp514x_reg_dump(sd);

	return 0;
}

static int tvp514x_detect(struct v4l2_subdev *sd,
		struct tvp514x_decoder *decoder)
{
	u8 chip_id_msb, chip_id_lsb, rom_ver;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	chip_id_msb = tvp514x_read_reg(sd, REG_CHIP_ID_MSB);
	chip_id_lsb = tvp514x_read_reg(sd, REG_CHIP_ID_LSB);
	rom_ver = tvp514x_read_reg(sd, REG_ROM_VERSION);

	v4l2_dbg(1, debug, sd,
		 "chip id detected msb:0x%x lsb:0x%x rom version:0x%x\n",
		 chip_id_msb, chip_id_lsb, rom_ver);
	if ((chip_id_msb != TVP514X_CHIP_ID_MSB)
		|| ((chip_id_lsb != TVP5146_CHIP_ID_LSB)
		&& (chip_id_lsb != TVP5147_CHIP_ID_LSB))) {
		/* We didn't read the values we expected, so this must not be
		 * an TVP5146/47.
		 */
		v4l2_err(sd, "chip id mismatch msb:0x%x lsb:0x%x\n",
				chip_id_msb, chip_id_lsb);
		return -ENODEV;
	}

	decoder->ver = rom_ver;

	v4l2_info(sd, "%s (Version - 0x%.2x) found at 0x%x (%s)\n",
			client->name, decoder->ver,
			client->addr << 1, client->adapter->name);
	return 0;
}

static int tvp514x_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	enum tvp514x_std current_std;
	enum tvp514x_input input_sel;
	u8 sync_lock_status, lock_mask;

	if (std_id == NULL)
		return -EINVAL;

	*std_id = V4L2_STD_UNKNOWN;

	/* query the current standard */
	current_std = tvp514x_query_current_std(sd);
	if (current_std == STD_INVALID)
		return 0;

	input_sel = decoder->input;

	switch (input_sel) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
		/*Need to add other interfaces*/
	default:
		return -EINVAL;
	}
	/* check whether signal is locked */
	sync_lock_status = tvp514x_read_reg(sd, REG_STATUS1);
	if (lock_mask != (sync_lock_status & lock_mask))
		return 0;	/* No input detected */

	*std_id = decoder->std_list[current_std].standard.id;

	v4l2_dbg(1, debug, sd, "Current STD: %s\n",
			decoder->std_list[current_std].standard.name);
	return 0;
}

static int tvp514x_s_std(struct v4l2_subdev *sd, v4l2_std_id std_id)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	int err, i;

	for (i = 0; i < decoder->num_stds; i++)
		if (std_id & decoder->std_list[i].standard.id)
			break;

	if ((i == decoder->num_stds) || (i == STD_INVALID))
		return -EINVAL;

	err = tvp514x_write_reg(sd, REG_VIDEO_STD,
				decoder->std_list[i].video_std);
	if (err)
		return err;

	decoder->current_std = i;
	decoder->tvp514x_regs[REG_VIDEO_STD].val =
		decoder->std_list[i].video_std;

	v4l2_dbg(1, debug, sd, "Standard set to: %s\n",
			decoder->std_list[i].standard.name);
	return 0;
}

static int tvp514x_s_routing(struct v4l2_subdev *sd,
				u32 input, u32 output, u32 config)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	int err;
	enum tvp514x_input input_sel;
	enum tvp514x_output output_sel;
	u8 sync_lock_status, lock_mask;
	int try_count = LOCK_RETRY_COUNT;

	if ((input >= INPUT_INVALID) ||
			(output >= OUTPUT_INVALID))
		/* Index out of bound */
		return -EINVAL;

	/*
	 * For the sequence streamon -> streamoff and again s_input
	 * it fails to lock the signal, since streamoff puts TVP514x
	 * into power off state which leads to failure in sub-sequent s_input.
	 *
	 * So power up the TVP514x device here, since it is important to lock
	 * the signal at this stage.
	 */
	if (!decoder->streaming)
		tvp514x_s_stream(sd, 1);

	input_sel = input;
	output_sel = output;

	err = tvp514x_write_reg(sd, REG_INPUT_SEL, input_sel);
	if (err)
		return err;

	output_sel |= tvp514x_read_reg(sd,
			REG_OUTPUT_FORMATTER1) & 0x7;
	err = tvp514x_write_reg(sd, REG_OUTPUT_FORMATTER1,
			output_sel);
	if (err)
		return err;

	decoder->tvp514x_regs[REG_INPUT_SEL].val = input_sel;
	decoder->tvp514x_regs[REG_OUTPUT_FORMATTER1].val = output_sel;

	/* Clear status */
	msleep(LOCK_RETRY_DELAY);
	err =
	    tvp514x_write_reg(sd, REG_CLEAR_LOST_LOCK, 0x01);
	if (err)
		return err;

	switch (input_sel) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
	/* Need to add other interfaces*/
	default:
		return -EINVAL;
	}

	while (try_count-- > 0) {
		/* Allow decoder to sync up with new input */
		msleep(LOCK_RETRY_DELAY);

		sync_lock_status = tvp514x_read_reg(sd,
				REG_STATUS1);
		if (lock_mask == (sync_lock_status & lock_mask))
			/* Input detected */
			break;
	}

	if (try_count < 0)
		return -EINVAL;

	decoder->input = input;
	decoder->output = output;

	v4l2_dbg(1, debug, sd, "Input set to: %d\n", input_sel);

	return 0;
}

static int
tvp514x_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qctrl)
{
	int err = -EINVAL;

	if (qctrl == NULL)
		return err;

	switch (qctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* Brightness supported is (0-255), */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
		/**
		 * Saturation and Contrast supported is -
		 *	Contrast: 0 - 255 (Default - 128)
		 *	Saturation: 0 - 255 (Default - 128)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_HUE:
		/* Hue Supported is -
		 *	Hue - -180 - +180 (Default - 0, Step - +180)
		 */
		err = v4l2_ctrl_query_fill(qctrl, -180, 180, 180, 0);
		break;
	case V4L2_CID_AUTOGAIN:
		/**
		 * Auto Gain supported is -
		 * 	0 - 1 (Default - 1)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 1, 1, 1);
		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", qctrl->id);
		return err;
	}

	v4l2_dbg(1, debug, sd, "Query Control:%s: Min - %d, Max - %d, Def - %d\n",
			qctrl->name, qctrl->minimum, qctrl->maximum,
			qctrl->default_value);

	return err;
}

static int
tvp514x_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);

	if (ctrl == NULL)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = decoder->tvp514x_regs[REG_BRIGHTNESS].val;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = decoder->tvp514x_regs[REG_CONTRAST].val;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = decoder->tvp514x_regs[REG_SATURATION].val;
		break;
	case V4L2_CID_HUE:
		ctrl->value = decoder->tvp514x_regs[REG_HUE].val;
		if (ctrl->value == 0x7F)
			ctrl->value = 180;
		else if (ctrl->value == 0x80)
			ctrl->value = -180;
		else
			ctrl->value = 0;

		break;
	case V4L2_CID_AUTOGAIN:
		ctrl->value = decoder->tvp514x_regs[REG_AFE_GAIN_CTRL].val;
		if ((ctrl->value & 0x3) == 3)
			ctrl->value = 1;
		else
			ctrl->value = 0;

		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", ctrl->id);
		return -EINVAL;
	}

	v4l2_dbg(1, debug, sd, "Get Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);
	return 0;
}

static int
tvp514x_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	int err = -EINVAL, value;

	if (ctrl == NULL)
		return err;

	value = ctrl->value;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid brightness setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp514x_write_reg(sd, REG_BRIGHTNESS,
				value);
		if (err)
			return err;

		decoder->tvp514x_regs[REG_BRIGHTNESS].val = value;
		break;
	case V4L2_CID_CONTRAST:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid contrast setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp514x_write_reg(sd, REG_CONTRAST, value);
		if (err)
			return err;

		decoder->tvp514x_regs[REG_CONTRAST].val = value;
		break;
	case V4L2_CID_SATURATION:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid saturation setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp514x_write_reg(sd, REG_SATURATION, value);
		if (err)
			return err;

		decoder->tvp514x_regs[REG_SATURATION].val = value;
		break;
	case V4L2_CID_HUE:
		if (value == 180)
			value = 0x7F;
		else if (value == -180)
			value = 0x80;
		else if (value == 0)
			value = 0;
		else {
			v4l2_err(sd, "invalid hue setting %d\n", ctrl->value);
			return -ERANGE;
		}
		err = tvp514x_write_reg(sd, REG_HUE, value);
		if (err)
			return err;

		decoder->tvp514x_regs[REG_HUE].val = value;
		break;
	case V4L2_CID_AUTOGAIN:
		if (value == 1)
			value = 0x0F;
		else if (value == 0)
			value = 0x0C;
		else {
			v4l2_err(sd, "invalid auto gain setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp514x_write_reg(sd, REG_AFE_GAIN_CTRL, value);
		if (err)
			return err;

		decoder->tvp514x_regs[REG_AFE_GAIN_CTRL].val = value;
		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", ctrl->id);
		return err;
	}

	v4l2_dbg(1, debug, sd, "Set Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);

	return err;
}

static int
tvp514x_enum_fmt_cap(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	if (fmt == NULL || fmt->index)
		return -EINVAL;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	/* only one format */
	fmt->flags = 0;
	strlcpy(fmt->description, "8-bit UYVY 4:2:2 Format",
					sizeof(fmt->description));
	fmt->pixelformat = V4L2_PIX_FMT_UYVY;
	return 0;
}

static int
tvp514x_fmt_cap(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	struct v4l2_pix_format *pix;
	enum tvp514x_std current_std;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pix = &f->fmt.pix;

	/* Calculate height and width based on current standard */
	current_std = decoder->current_std;

	pix->pixelformat = V4L2_PIX_FMT_UYVY;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;
	pix->field = V4L2_FIELD_INTERLACED;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	v4l2_dbg(1, debug, sd, "FMT: bytesperline - %d"
			"Width - %d, Height - %d\n",
			pix->bytesperline,
			pix->width, pix->height);
	return 0;
}

static int
tvp514x_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	struct v4l2_captureparm *cparm;
	enum tvp514x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	/* get the current standard */
	current_std = decoder->current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

static int
tvp514x_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp514x_decoder *decoder = to_decoder(sd);
	struct v4l2_fract *timeperframe;
	enum tvp514x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = decoder->current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

static int tvp514x_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tvp514x_decoder *decoder = to_decoder(sd);

	if (decoder->streaming == enable)
		return 0;

	switch (enable) {
	case 0:
	{
		/* Power Down Sequence */
		err = tvp514x_write_reg(sd, REG_OPERATION_MODE, 0x01);
		if (err) {
			v4l2_err(sd, "Unable to turn off decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	case 1:
	{
		struct tvp514x_reg *int_seq = (struct tvp514x_reg *)
				client->driver->id_table->driver_data;

		/* Power Up Sequence */
		err = tvp514x_write_regs(sd, int_seq);
		if (err) {
			v4l2_err(sd, "Unable to turn on decoder\n");
			return err;
		}
		/* Detect if not already detected */
		err = tvp514x_detect(sd, decoder);
		if (err) {
			v4l2_err(sd, "Unable to detect decoder\n");
			return err;
		}
		err = tvp514x_configure(sd, decoder);
		if (err) {
			v4l2_err(sd, "Unable to configure decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	default:
		err = -ENODEV;
		break;
	}

	return err;
}

static const struct v4l2_subdev_core_ops tvp514x_core_ops = {
	.queryctrl = tvp514x_queryctrl,
	.g_ctrl = tvp514x_g_ctrl,
	.s_ctrl = tvp514x_s_ctrl,
	.s_std = tvp514x_s_std,
};

static const struct v4l2_subdev_video_ops tvp514x_video_ops = {
	.s_routing = tvp514x_s_routing,
	.querystd = tvp514x_querystd,
	.enum_fmt = tvp514x_enum_fmt_cap,
	.g_fmt = tvp514x_fmt_cap,
	.try_fmt = tvp514x_fmt_cap,
	.s_fmt = tvp514x_fmt_cap,
	.g_parm = tvp514x_g_parm,
	.s_parm = tvp514x_s_parm,
	.s_stream = tvp514x_s_stream,
};

static const struct v4l2_subdev_ops tvp514x_ops = {
	.core = &tvp514x_core_ops,
	.video = &tvp514x_video_ops,
};

static struct tvp514x_decoder tvp514x_dev = {
	.streaming = 0,
	.current_std = STD_NTSC_MJ,
	.std_list = tvp514x_std_list,
	.num_stds = ARRAY_SIZE(tvp514x_std_list),

};

static int
tvp514x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tvp514x_decoder *decoder;
	struct v4l2_subdev *sd;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!client->dev.platform_data) {
		v4l2_err(client, "No platform data!!\n");
		return -ENODEV;
	}

	decoder = kzalloc(sizeof(*decoder), GFP_KERNEL);
	if (!decoder)
		return -ENOMEM;

	/* Initialize the tvp514x_decoder with default configuration */
	*decoder = tvp514x_dev;
	/* Copy default register configuration */
	memcpy(decoder->tvp514x_regs, tvp514x_reg_list_default,
			sizeof(tvp514x_reg_list_default));

	/* Copy board specific information here */
	decoder->pdata = client->dev.platform_data;

	/**
	 * Fetch platform specific data, and configure the
	 * tvp514x_reg_list[] accordingly. Since this is one
	 * time configuration, no need to preserve.
	 */
	decoder->tvp514x_regs[REG_OUTPUT_FORMATTER2].val |=
		(decoder->pdata->clk_polarity << 1);
	decoder->tvp514x_regs[REG_SYNC_CONTROL].val |=
		((decoder->pdata->hs_polarity << 2) |
		 (decoder->pdata->vs_polarity << 3));
	/* Set default standard to auto */
	decoder->tvp514x_regs[REG_VIDEO_STD].val =
		VIDEO_STD_AUTO_SWITCH_BIT;

	/* Register with V4L2 layer as slave device */
	sd = &decoder->sd;
	v4l2_i2c_subdev_init(sd, client, &tvp514x_ops);

	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);

	return 0;

}

static int tvp514x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tvp514x_decoder *decoder = to_decoder(sd);

	v4l2_device_unregister_subdev(sd);
	kfree(decoder);
	return 0;
}
/* TVP5146 Init/Power on Sequence */
static const struct tvp514x_reg tvp5146_init_reg_seq[] = {
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x02},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0x80},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_TERM, 0, 0},
};

/* TVP5147 Init/Power on Sequence */
static const struct tvp514x_reg tvp5147_init_reg_seq[] =	{
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x02},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0x80},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x16},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xA0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x16},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_TERM, 0, 0},
};

/* TVP5146M2/TVP5147M1 Init/Power on Sequence */
static const struct tvp514x_reg tvp514xm_init_reg_seq[] = {
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_TERM, 0, 0},
};

static const struct i2c_device_id tvp514x_id[] = {
	{"tvp5146", (unsigned long)tvp5146_init_reg_seq},
	{"tvp5146m2", (unsigned long)tvp514xm_init_reg_seq},
	{"tvp5147", (unsigned long)tvp5147_init_reg_seq},
	{"tvp5147m1", (unsigned long)tvp514xm_init_reg_seq},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp514x_id);

static struct i2c_driver tvp514x_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TVP514X_MODULE_NAME,
	},
	.probe = tvp514x_probe,
	.remove = tvp514x_remove,
	.id_table = tvp514x_id,
};

static int __init tvp514x_init(void)
{
	return i2c_add_driver(&tvp514x_driver);
}

static void __exit tvp514x_exit(void)
{
	i2c_del_driver(&tvp514x_driver);
}

module_init(tvp514x_init);
module_exit(tvp514x_exit);
