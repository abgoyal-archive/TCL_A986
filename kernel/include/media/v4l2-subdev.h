

#ifndef _V4L2_SUBDEV_H
#define _V4L2_SUBDEV_H

#include <media/v4l2-common.h>
#include <media/v4l2-mediabus.h>

/* generic v4l2_device notify callback notification values */
#define V4L2_SUBDEV_IR_RX_NOTIFY		_IOW('v', 0, u32)
#define V4L2_SUBDEV_IR_RX_FIFO_SERVICE_REQ	0x00000001
#define V4L2_SUBDEV_IR_RX_END_OF_RX_DETECTED	0x00000002
#define V4L2_SUBDEV_IR_RX_HW_FIFO_OVERRUN	0x00000004
#define V4L2_SUBDEV_IR_RX_SW_FIFO_OVERRUN	0x00000008

#define V4L2_SUBDEV_IR_TX_NOTIFY		_IOW('v', 1, u32)
#define V4L2_SUBDEV_IR_TX_FIFO_SERVICE_REQ	0x00000001

struct v4l2_device;
struct v4l2_subdev;
struct tuner_setup;

/* decode_vbi_line */
struct v4l2_decode_vbi_line {
	u32 is_second_field;	/* Set to 0 for the first (odd) field,
				   set to 1 for the second (even) field. */
	u8 *p; 			/* Pointer to the sliced VBI data from the decoder.
				   On exit points to the start of the payload. */
	u32 line;		/* Line number of the sliced VBI data (1-23) */
	u32 type;		/* VBI service type (V4L2_SLICED_*). 0 if no service found */
};



struct v4l2_subdev_core_ops {
	int (*g_chip_ident)(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip);
	int (*log_status)(struct v4l2_subdev *sd);
	int (*s_config)(struct v4l2_subdev *sd, int irq, void *platform_data);
	int (*init)(struct v4l2_subdev *sd, u32 val);
	int (*load_fw)(struct v4l2_subdev *sd);
	int (*reset)(struct v4l2_subdev *sd, u32 val);
	int (*s_gpio)(struct v4l2_subdev *sd, u32 val);
	int (*queryctrl)(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc);
	int (*g_ctrl)(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
	int (*s_ctrl)(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
	int (*g_ext_ctrls)(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
	int (*s_ext_ctrls)(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
	int (*try_ext_ctrls)(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
	int (*querymenu)(struct v4l2_subdev *sd, struct v4l2_querymenu *qm);
	int (*s_std)(struct v4l2_subdev *sd, v4l2_std_id norm);
	long (*ioctl)(struct v4l2_subdev *sd, unsigned int cmd, void *arg);
#ifdef CONFIG_VIDEO_ADV_DEBUG
	int (*g_register)(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg);
	int (*s_register)(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg);
#endif
	int (*s_power)(struct v4l2_subdev *sd, int on);
};

struct v4l2_subdev_tuner_ops {
	int (*s_mode)(struct v4l2_subdev *sd, enum v4l2_tuner_type);
	int (*s_radio)(struct v4l2_subdev *sd);
	int (*s_frequency)(struct v4l2_subdev *sd, struct v4l2_frequency *freq);
	int (*g_frequency)(struct v4l2_subdev *sd, struct v4l2_frequency *freq);
	int (*g_tuner)(struct v4l2_subdev *sd, struct v4l2_tuner *vt);
	int (*s_tuner)(struct v4l2_subdev *sd, struct v4l2_tuner *vt);
	int (*g_modulator)(struct v4l2_subdev *sd, struct v4l2_modulator *vm);
	int (*s_modulator)(struct v4l2_subdev *sd, struct v4l2_modulator *vm);
	int (*s_type_addr)(struct v4l2_subdev *sd, struct tuner_setup *type);
	int (*s_config)(struct v4l2_subdev *sd, const struct v4l2_priv_tun_config *config);
};

struct v4l2_subdev_audio_ops {
	int (*s_clock_freq)(struct v4l2_subdev *sd, u32 freq);
	int (*s_i2s_clock_freq)(struct v4l2_subdev *sd, u32 freq);
	int (*s_routing)(struct v4l2_subdev *sd, u32 input, u32 output, u32 config);
	int (*s_stream)(struct v4l2_subdev *sd, int enable);
};

struct v4l2_subdev_video_ops {
	int (*s_routing)(struct v4l2_subdev *sd, u32 input, u32 output, u32 config);
	int (*s_crystal_freq)(struct v4l2_subdev *sd, u32 freq, u32 flags);
	int (*s_std_output)(struct v4l2_subdev *sd, v4l2_std_id std);
	int (*querystd)(struct v4l2_subdev *sd, v4l2_std_id *std);
	int (*g_input_status)(struct v4l2_subdev *sd, u32 *status);
	int (*s_stream)(struct v4l2_subdev *sd, int enable);
	int (*enum_fmt)(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc);
	int (*g_fmt)(struct v4l2_subdev *sd, struct v4l2_format *fmt);
	int (*try_fmt)(struct v4l2_subdev *sd, struct v4l2_format *fmt);
	int (*s_fmt)(struct v4l2_subdev *sd, struct v4l2_format *fmt);
	int (*cropcap)(struct v4l2_subdev *sd, struct v4l2_cropcap *cc);
	int (*g_crop)(struct v4l2_subdev *sd, struct v4l2_crop *crop);
	int (*s_crop)(struct v4l2_subdev *sd, struct v4l2_crop *crop);
	int (*g_parm)(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
	int (*s_parm)(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
	int (*enum_framesizes)(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize);
	int (*enum_frameintervals)(struct v4l2_subdev *sd, struct v4l2_frmivalenum *fival);
	int (*enum_dv_presets) (struct v4l2_subdev *sd,
			struct v4l2_dv_enum_preset *preset);
	int (*s_dv_preset)(struct v4l2_subdev *sd,
			struct v4l2_dv_preset *preset);
	int (*query_dv_preset)(struct v4l2_subdev *sd,
			struct v4l2_dv_preset *preset);
	int (*s_dv_timings)(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings);
	int (*g_dv_timings)(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings);
	int (*enum_mbus_fmt)(struct v4l2_subdev *sd, unsigned int index,
			     enum v4l2_mbus_pixelcode *code);
	int (*g_mbus_fmt)(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *fmt);
	int (*try_mbus_fmt)(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt);
	int (*s_mbus_fmt)(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *fmt);
};

struct v4l2_subdev_vbi_ops {
	int (*decode_vbi_line)(struct v4l2_subdev *sd, struct v4l2_decode_vbi_line *vbi_line);
	int (*s_vbi_data)(struct v4l2_subdev *sd, const struct v4l2_sliced_vbi_data *vbi_data);
	int (*g_vbi_data)(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_data *vbi_data);
	int (*g_sliced_vbi_cap)(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_cap *cap);
	int (*s_raw_fmt)(struct v4l2_subdev *sd, struct v4l2_vbi_format *fmt);
	int (*g_sliced_fmt)(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_format *fmt);
	int (*s_sliced_fmt)(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_format *fmt);
};

struct v4l2_subdev_sensor_ops {
	int (*g_skip_top_lines)(struct v4l2_subdev *sd, u32 *lines);
};


enum v4l2_subdev_ir_mode {
	V4L2_SUBDEV_IR_MODE_PULSE_WIDTH, /* space & mark widths in nanosecs */
};

/* Data format of data read or written for V4L2_SUBDEV_IR_MODE_PULSE_WIDTH */
#define V4L2_SUBDEV_IR_PULSE_MAX_WIDTH_NS	0x7fffffff
#define V4L2_SUBDEV_IR_PULSE_LEVEL_MASK		0x80000000
#define V4L2_SUBDEV_IR_PULSE_RX_SEQ_END		0xffffffff

struct v4l2_subdev_ir_parameters {
	/* Either Rx or Tx */
	unsigned int bytes_per_data_element; /* of data in read or write call */
	enum v4l2_subdev_ir_mode mode;

	bool enable;
	bool interrupt_enable;
	bool shutdown; /* true: set hardware to low/no power, false: normal */

	bool modulation;           /* true: uses carrier, false: baseband */
	u32 max_pulse_width;       /* ns,      valid only for baseband signal */
	unsigned int carrier_freq; /* Hz,      valid only for modulated signal*/
	unsigned int duty_cycle;   /* percent, valid only for modulated signal*/
	bool invert;		   /* logically invert sense of mark/space */

	/* Rx only */
	u32 noise_filter_min_width;       /* ns, min time of a valid pulse */
	unsigned int carrier_range_lower; /* Hz, valid only for modulated sig */
	unsigned int carrier_range_upper; /* Hz, valid only for modulated sig */
	u32 resolution;                   /* ns */
};

struct v4l2_subdev_ir_ops {
	/* Common to receiver and transmitter */
	int (*interrupt_service_routine)(struct v4l2_subdev *sd,
						u32 status, bool *handled);

	/* Receiver */
	int (*rx_read)(struct v4l2_subdev *sd, u8 *buf, size_t count,
				ssize_t *num);

	int (*rx_g_parameters)(struct v4l2_subdev *sd,
				struct v4l2_subdev_ir_parameters *params);
	int (*rx_s_parameters)(struct v4l2_subdev *sd,
				struct v4l2_subdev_ir_parameters *params);

	/* Transmitter */
	int (*tx_write)(struct v4l2_subdev *sd, u8 *buf, size_t count,
				ssize_t *num);

	int (*tx_g_parameters)(struct v4l2_subdev *sd,
				struct v4l2_subdev_ir_parameters *params);
	int (*tx_s_parameters)(struct v4l2_subdev *sd,
				struct v4l2_subdev_ir_parameters *params);
};

struct v4l2_subdev_ops {
	const struct v4l2_subdev_core_ops	*core;
	const struct v4l2_subdev_tuner_ops	*tuner;
	const struct v4l2_subdev_audio_ops	*audio;
	const struct v4l2_subdev_video_ops	*video;
	const struct v4l2_subdev_vbi_ops	*vbi;
	const struct v4l2_subdev_ir_ops		*ir;
	const struct v4l2_subdev_sensor_ops	*sensor;
};

#define V4L2_SUBDEV_NAME_SIZE 32

/* Set this flag if this subdev is a i2c device. */
#define V4L2_SUBDEV_FL_IS_I2C (1U << 0)
/* Set this flag if this subdev is a spi device. */
#define V4L2_SUBDEV_FL_IS_SPI (1U << 1)

struct v4l2_subdev {
	struct list_head list;
	struct module *owner;
	u32 flags;
	struct v4l2_device *v4l2_dev;
	const struct v4l2_subdev_ops *ops;
	/* name must be unique */
	char name[V4L2_SUBDEV_NAME_SIZE];
	/* can be used to group similar subdevs, value is driver-specific */
	u32 grp_id;
	/* pointer to private data */
	void *priv;
};

static inline void v4l2_set_subdevdata(struct v4l2_subdev *sd, void *p)
{
	sd->priv = p;
}

static inline void *v4l2_get_subdevdata(const struct v4l2_subdev *sd)
{
	return sd->priv;
}

static inline void v4l2_subdev_init(struct v4l2_subdev *sd,
					const struct v4l2_subdev_ops *ops)
{
	INIT_LIST_HEAD(&sd->list);
	/* ops->core MUST be set */
	BUG_ON(!ops || !ops->core);
	sd->ops = ops;
	sd->v4l2_dev = NULL;
	sd->flags = 0;
	sd->name[0] = '\0';
	sd->grp_id = 0;
	sd->priv = NULL;
}

#define v4l2_subdev_call(sd, o, f, args...)				\
	(!(sd) ? -ENODEV : (((sd)->ops->o && (sd)->ops->o->f) ?	\
		(sd)->ops->o->f((sd) , ##args) : -ENOIOCTLCMD))

/* Send a notification to v4l2_device. */
#define v4l2_subdev_notify(sd, notification, arg)			   \
	((!(sd) || !(sd)->v4l2_dev || !(sd)->v4l2_dev->notify) ? -ENODEV : \
	 (sd)->v4l2_dev->notify((sd), (notification), (arg)))

#endif
