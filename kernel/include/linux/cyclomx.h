
#ifndef	_CYCLOMX_H
#define	_CYCLOMX_H

#include <linux/wanrouter.h>
#include <linux/spinlock.h>

#ifdef	__KERNEL__
/* Kernel Interface */

#include <linux/cycx_drv.h>	/* Cyclom 2X support module API definitions */
#include <linux/cycx_cfm.h>	/* Cyclom 2X firmware module definitions */
#ifdef CONFIG_CYCLOMX_X25
#include <linux/cycx_x25.h>
#endif

struct cycx_device {
	char devname[WAN_DRVNAME_SZ + 1];/* card name */
	struct cycx_hw hw;		/* hardware configuration */
	struct wan_device wandev;	/* WAN device data space */
	u32 state_tick;			/* link state timestamp */
	spinlock_t lock;
	char in_isr;			/* interrupt-in-service flag */
	char buff_int_mode_unbusy;      /* flag for carrying out dev_tint */
	wait_queue_head_t wait_stats;  /* to wait for the STATS indication */
	void __iomem *mbox;			/* -> mailbox */
	void (*isr)(struct cycx_device* card);	/* interrupt service routine */
	int (*exec)(struct cycx_device* card, void* u_cmd, void* u_data);
	union {
#ifdef CONFIG_CYCLOMX_X25
		struct { /* X.25 specific data */
			u32 lo_pvc;
			u32 hi_pvc;
			u32 lo_svc;
			u32 hi_svc;
			struct cycx_x25_stats stats;
			spinlock_t lock;
			u32 connection_keys;
		} x;
#endif
	} u;
};

/* Public Functions */
void cycx_set_state(struct cycx_device *card, int state);

#ifdef CONFIG_CYCLOMX_X25
int cycx_x25_wan_init(struct cycx_device *card, wandev_conf_t *conf);
#endif
#endif	/* __KERNEL__ */
#endif	/* _CYCLOMX_H */
