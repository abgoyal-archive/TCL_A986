
#ifndef __PVRUSB2_HDW_H
#define __PVRUSB2_HDW_H

#include <linux/usb.h>
#include <linux/videodev2.h>
#include "pvrusb2-io.h"
#include "pvrusb2-ctrl.h"


#define PVR2_CID_STDENUM 1
#define PVR2_CID_STDCUR 2
#define PVR2_CID_STDAVAIL 3
#define PVR2_CID_INPUT 4
#define PVR2_CID_AUDIOMODE 5
#define PVR2_CID_FREQUENCY 6
#define PVR2_CID_HRES 7
#define PVR2_CID_VRES 8
#define PVR2_CID_CROPL 9
#define PVR2_CID_CROPT 10
#define PVR2_CID_CROPW 11
#define PVR2_CID_CROPH 12
#define PVR2_CID_CROPCAPPAN 13
#define PVR2_CID_CROPCAPPAD 14
#define PVR2_CID_CROPCAPBL 15
#define PVR2_CID_CROPCAPBT 16
#define PVR2_CID_CROPCAPBW 17
#define PVR2_CID_CROPCAPBH 18

/* Legal values for the INPUT state variable */
#define PVR2_CVAL_INPUT_TV 0
#define PVR2_CVAL_INPUT_DTV 1
#define PVR2_CVAL_INPUT_COMPOSITE 2
#define PVR2_CVAL_INPUT_SVIDEO 3
#define PVR2_CVAL_INPUT_RADIO 4

enum pvr2_config {
	pvr2_config_empty,    /* No configuration */
	pvr2_config_mpeg,     /* Encoded / compressed video */
	pvr2_config_vbi,      /* Standard vbi info */
	pvr2_config_pcm,      /* Audio raw pcm stream */
	pvr2_config_rawvideo, /* Video raw frames */
};

enum pvr2_v4l_type {
	pvr2_v4l_type_video,
	pvr2_v4l_type_vbi,
	pvr2_v4l_type_radio,
};

#define PVR2_STATE_NONE 0
#define PVR2_STATE_DEAD 1
#define PVR2_STATE_COLD 2
#define PVR2_STATE_WARM 3
#define PVR2_STATE_ERROR 4
#define PVR2_STATE_READY 5
#define PVR2_STATE_RUN 6

/* Translate configuration enum to a string label */
const char *pvr2_config_get_name(enum pvr2_config);

struct pvr2_hdw;

struct pvr2_hdw *pvr2_hdw_create(struct usb_interface *intf,
				 const struct usb_device_id *devid);

int pvr2_hdw_initialize(struct pvr2_hdw *,
			void (*callback_func)(void *),
			void *callback_data);

/* Destroy hardware interaction structure */
void pvr2_hdw_destroy(struct pvr2_hdw *);

/* Return true if in the ready (normal) state */
int pvr2_hdw_dev_ok(struct pvr2_hdw *);

int pvr2_hdw_get_unit_number(struct pvr2_hdw *);

/* Get pointer to underlying USB device */
struct usb_device *pvr2_hdw_get_dev(struct pvr2_hdw *);

/* Retrieve serial number of device */
unsigned long pvr2_hdw_get_sn(struct pvr2_hdw *);

/* Retrieve bus location info of device */
const char *pvr2_hdw_get_bus_info(struct pvr2_hdw *);

/* Retrieve per-instance string identifier for this specific device */
const char *pvr2_hdw_get_device_identifier(struct pvr2_hdw *);

/* Called when hardware has been unplugged */
void pvr2_hdw_disconnect(struct pvr2_hdw *);

/* Get the number of defined controls */
unsigned int pvr2_hdw_get_ctrl_count(struct pvr2_hdw *);

/* Retrieve a control handle given its index (0..count-1) */
struct pvr2_ctrl *pvr2_hdw_get_ctrl_by_index(struct pvr2_hdw *,unsigned int);

/* Retrieve a control handle given its internal ID (if any) */
struct pvr2_ctrl *pvr2_hdw_get_ctrl_by_id(struct pvr2_hdw *,unsigned int);

/* Retrieve a control handle given its V4L ID (if any) */
struct pvr2_ctrl *pvr2_hdw_get_ctrl_v4l(struct pvr2_hdw *,unsigned int ctl_id);

/* Retrieve a control handle given its immediate predecessor V4L ID (if any) */
struct pvr2_ctrl *pvr2_hdw_get_ctrl_nextv4l(struct pvr2_hdw *,
					    unsigned int ctl_id);

/* Commit all control changes made up to this point */
int pvr2_hdw_commit_ctl(struct pvr2_hdw *);

unsigned int pvr2_hdw_get_input_available(struct pvr2_hdw *);

unsigned int pvr2_hdw_get_input_allowed(struct pvr2_hdw *);

int pvr2_hdw_set_input_allowed(struct pvr2_hdw *,
			       unsigned int change_mask,
			       unsigned int change_val);

/* Return name for this driver instance */
const char *pvr2_hdw_get_driver_name(struct pvr2_hdw *);

/* Mark tuner status stale so that it will be re-fetched */
void pvr2_hdw_execute_tuner_poll(struct pvr2_hdw *);

/* Return information about the tuner */
int pvr2_hdw_get_tuner_status(struct pvr2_hdw *,struct v4l2_tuner *);

/* Return information about cropping capabilities */
int pvr2_hdw_get_cropcap(struct pvr2_hdw *, struct v4l2_cropcap *);

/* Query device and see if it thinks it is on a high-speed USB link */
int pvr2_hdw_is_hsm(struct pvr2_hdw *);

/* Return a string token representative of the hardware type */
const char *pvr2_hdw_get_type(struct pvr2_hdw *);

/* Return a single line description of the hardware type */
const char *pvr2_hdw_get_desc(struct pvr2_hdw *);

/* Turn streaming on/off */
int pvr2_hdw_set_streaming(struct pvr2_hdw *,int);

/* Find out if streaming is on */
int pvr2_hdw_get_streaming(struct pvr2_hdw *);

/* Retrieve driver overall state */
int pvr2_hdw_get_state(struct pvr2_hdw *);

/* Configure the type of stream to generate */
int pvr2_hdw_set_stream_type(struct pvr2_hdw *, enum pvr2_config);

/* Get handle to video output stream */
struct pvr2_stream *pvr2_hdw_get_video_stream(struct pvr2_hdw *);

/* Emit a video standard struct */
int pvr2_hdw_get_stdenum_value(struct pvr2_hdw *hdw,struct v4l2_standard *std,
			       unsigned int idx);

void pvr2_hdw_cpufw_set_enabled(struct pvr2_hdw *,
				int mode, /* 0=8KB FX2, 1=16KB FX2, 2=PROM */
				int enable_flag);

/* Return true if we're in a mode for retrieval CPU firmware */
int pvr2_hdw_cpufw_get_enabled(struct pvr2_hdw *);

int pvr2_hdw_cpufw_get(struct pvr2_hdw *,unsigned int offs,
		       char *buf,unsigned int cnt);

/* Retrieve a previously stored v4l minor device number */
int pvr2_hdw_v4l_get_minor_number(struct pvr2_hdw *,enum pvr2_v4l_type index);

/* Store a v4l minor device number */
void pvr2_hdw_v4l_store_minor_number(struct pvr2_hdw *,
				     enum pvr2_v4l_type index,int);

int pvr2_hdw_register_access(struct pvr2_hdw *,
			     struct v4l2_dbg_match *match, u64 reg_id,
			     int setFl, u64 *val_ptr);


int pvr2_send_request(struct pvr2_hdw *,
		      void *write_ptr,unsigned int write_len,
		      void *read_ptr,unsigned int read_len);

/* Slightly higher level device communication functions. */
int pvr2_write_register(struct pvr2_hdw *, u16, u32);

void pvr2_hdw_render_useless(struct pvr2_hdw *);

/* Set / clear 8051's reset bit */
void pvr2_hdw_cpureset_assert(struct pvr2_hdw *,int);

/* Execute a USB-commanded device reset */
void pvr2_hdw_device_reset(struct pvr2_hdw *);

/* Reset worker's error trapping circuit breaker */
int pvr2_hdw_untrip(struct pvr2_hdw *);

int pvr2_hdw_cmd_deep_reset(struct pvr2_hdw *);

/* Execute simple reset command */
int pvr2_hdw_cmd_powerup(struct pvr2_hdw *);

/* suspend */
int pvr2_hdw_cmd_powerdown(struct pvr2_hdw *);

/* Order decoder to reset */
int pvr2_hdw_cmd_decoder_reset(struct pvr2_hdw *);

/* Direct manipulation of GPIO bits */
int pvr2_hdw_gpio_get_dir(struct pvr2_hdw *hdw,u32 *);
int pvr2_hdw_gpio_get_out(struct pvr2_hdw *hdw,u32 *);
int pvr2_hdw_gpio_get_in(struct pvr2_hdw *hdw,u32 *);
int pvr2_hdw_gpio_chg_dir(struct pvr2_hdw *hdw,u32 msk,u32 val);
int pvr2_hdw_gpio_chg_out(struct pvr2_hdw *hdw,u32 msk,u32 val);

/* This data structure is specifically for the next function... */
struct pvr2_hdw_debug_info {
	int big_lock_held;
	int ctl_lock_held;
	int flag_disconnected;
	int flag_init_ok;
	int flag_ok;
	int fw1_state;
	int flag_decoder_missed;
	int flag_tripped;
	int state_encoder_ok;
	int state_encoder_run;
	int state_decoder_run;
	int state_decoder_ready;
	int state_usbstream_run;
	int state_decoder_quiescent;
	int state_pipeline_config;
	int state_pipeline_req;
	int state_pipeline_pause;
	int state_pipeline_idle;
	int cmd_debug_state;
	int cmd_debug_write_len;
	int cmd_debug_read_len;
	int cmd_debug_write_pend;
	int cmd_debug_read_pend;
	int cmd_debug_timeout;
	int cmd_debug_rstatus;
	int cmd_debug_wstatus;
	unsigned char cmd_code;
};

void pvr2_hdw_get_debug_info_unlocked(const struct pvr2_hdw *hdw,
				      struct pvr2_hdw_debug_info *);

void pvr2_hdw_get_debug_info_locked(struct pvr2_hdw *hdw,
				    struct pvr2_hdw_debug_info *);

unsigned int pvr2_hdw_state_report(struct pvr2_hdw *hdw,
				   char *buf_ptr,unsigned int buf_size);

/* Cause modules to log their state once */
void pvr2_hdw_trigger_module_log(struct pvr2_hdw *hdw);

int pvr2_upload_firmware2(struct pvr2_hdw *hdw);

#endif /* __PVRUSB2_HDW_H */

