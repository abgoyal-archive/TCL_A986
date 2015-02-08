

#ifndef _DVBDEV_H_
#define _DVBDEV_H_

#include <linux/types.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/list.h>

#define DVB_MAJOR 212

#if defined(CONFIG_DVB_MAX_ADAPTERS) && CONFIG_DVB_MAX_ADAPTERS > 0
  #define DVB_MAX_ADAPTERS CONFIG_DVB_MAX_ADAPTERS
#else
  #define DVB_MAX_ADAPTERS 8
#endif

#define DVB_UNSET (-1)

#define DVB_DEVICE_VIDEO      0
#define DVB_DEVICE_AUDIO      1
#define DVB_DEVICE_SEC        2
#define DVB_DEVICE_FRONTEND   3
#define DVB_DEVICE_DEMUX      4
#define DVB_DEVICE_DVR        5
#define DVB_DEVICE_CA         6
#define DVB_DEVICE_NET        7
#define DVB_DEVICE_OSD        8

#define DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr) \
	static short adapter_nr[] = \
		{[0 ... (DVB_MAX_ADAPTERS - 1)] = DVB_UNSET }; \
	module_param_array(adapter_nr, short, NULL, 0444); \
	MODULE_PARM_DESC(adapter_nr, "DVB adapter numbers")

struct dvb_frontend;

struct dvb_adapter {
	int num;
	struct list_head list_head;
	struct list_head device_list;
	const char *name;
	u8 proposed_mac [6];
	void* priv;

	struct device *device;

	struct module *module;

	int mfe_shared;			/* indicates mutually exclusive frontends */
	struct dvb_device *mfe_dvbdev;	/* frontend device in use */
	struct mutex mfe_lock;		/* access lock for thread creation */

	/* Allow the adapter/bridge driver to perform an action before and/or
	 * after the core handles an ioctl:
	 *
	 * DVB_FE_IOCTL_PRE indicates that the ioctl has not yet been handled.
	 * DVB_FE_IOCTL_POST indicates that the ioctl has been handled.
	 *
	 * When DVB_FE_IOCTL_PRE is passed to the callback as the stage arg:
	 *
	 * return 0 to allow dvb-core to handle the ioctl.
	 * return a positive int to prevent dvb-core from handling the ioctl,
	 * 	and exit without error.
	 * return a negative int to prevent dvb-core from handling the ioctl,
	 * 	and return that value as an error.
	 *
	 * When DVB_FE_IOCTL_POST is passed to the callback as the stage arg:
	 *
	 * return 0 to allow the dvb_frontend ioctl handler to exit normally.
	 * return a negative int to cause the dvb_frontend ioctl handler to
	 * 	return that value as an error.
	 */
#define DVB_FE_IOCTL_PRE 0
#define DVB_FE_IOCTL_POST 1
	int (*fe_ioctl_override)(struct dvb_frontend *fe,
				 unsigned int cmd, void *parg,
				 unsigned int stage);
};


struct dvb_device {
	struct list_head list_head;
	const struct file_operations *fops;
	struct dvb_adapter *adapter;
	int type;
	int minor;
	u32 id;

	/* in theory, 'users' can vanish now,
	   but I don't want to change too much now... */
	int readers;
	int writers;
	int users;

	wait_queue_head_t	  wait_queue;
	/* don't really need those !? -- FIXME: use video_usercopy  */
	int (*kernel_ioctl)(struct file *file, unsigned int cmd, void *arg);

	void *priv;
};


extern int dvb_register_adapter(struct dvb_adapter *adap, const char *name,
				struct module *module, struct device *device,
				short *adapter_nums);
extern int dvb_unregister_adapter (struct dvb_adapter *adap);

extern int dvb_register_device (struct dvb_adapter *adap,
				struct dvb_device **pdvbdev,
				const struct dvb_device *template,
				void *priv,
				int type);

extern void dvb_unregister_device (struct dvb_device *dvbdev);

extern int dvb_generic_open (struct inode *inode, struct file *file);
extern int dvb_generic_release (struct inode *inode, struct file *file);
extern long dvb_generic_ioctl (struct file *file,
			      unsigned int cmd, unsigned long arg);


extern int dvb_usercopy(struct file *file, unsigned int cmd, unsigned long arg,
			    int (*func)(struct file *file, unsigned int cmd, void *arg));

/** generic DVB attach function. */
#ifdef CONFIG_MEDIA_ATTACH
#define dvb_attach(FUNCTION, ARGS...) ({ \
	void *__r = NULL; \
	typeof(&FUNCTION) __a = symbol_request(FUNCTION); \
	if (__a) { \
		__r = (void *) __a(ARGS); \
		if (__r == NULL) \
			symbol_put(FUNCTION); \
	} else { \
		printk(KERN_ERR "DVB: Unable to find symbol "#FUNCTION"()\n"); \
	} \
	__r; \
})

#else
#define dvb_attach(FUNCTION, ARGS...) ({ \
	FUNCTION(ARGS); \
})

#endif

#endif /* #ifndef _DVBDEV_H_ */
