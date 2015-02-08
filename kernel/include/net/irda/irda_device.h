


#ifndef IRDA_DEVICE_H
#define IRDA_DEVICE_H

#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>		/* struct sk_buff */
#include <linux/irda.h>
#include <linux/types.h>

#include <net/pkt_sched.h>
#include <net/irda/irda.h>
#include <net/irda/qos.h>		/* struct qos_info */
#include <net/irda/irqueue.h>		/* irda_queue_t */

/* A few forward declarations (to make compiler happy) */
struct irlap_cb;

/* Some non-standard interface flags (should not conflict with any in if.h) */
#define IFF_SIR 	0x0001 /* Supports SIR speeds */
#define IFF_MIR 	0x0002 /* Supports MIR speeds */
#define IFF_FIR 	0x0004 /* Supports FIR speeds */
#define IFF_VFIR        0x0008 /* Supports VFIR speeds */
#define IFF_PIO   	0x0010 /* Supports PIO transfer of data */
#define IFF_DMA		0x0020 /* Supports DMA transfer of data */
#define IFF_SHM         0x0040 /* Supports shared memory data transfers */
#define IFF_DONGLE      0x0080 /* Interface has a dongle attached */
#define IFF_AIR         0x0100 /* Supports Advanced IR (AIR) standards */

#define IO_XMIT 0x01
#define IO_RECV 0x02

typedef enum {
	IRDA_IRLAP, /* IrDA mode, and deliver to IrLAP */
	IRDA_RAW,   /* IrDA mode */
	SHARP_ASK,
	TV_REMOTE,  /* Also known as Consumer Electronics IR */
} INFRARED_MODE;

typedef enum {
	IRDA_TASK_INIT,        /* All tasks are initialized with this state */
	IRDA_TASK_DONE,        /* Signals that the task is finished */
	IRDA_TASK_WAIT,
	IRDA_TASK_WAIT1,
	IRDA_TASK_WAIT2,
	IRDA_TASK_WAIT3,
	IRDA_TASK_CHILD_INIT,  /* Initializing child task */
	IRDA_TASK_CHILD_WAIT,  /* Waiting for child task to finish */
	IRDA_TASK_CHILD_DONE   /* Child task is finished */
} IRDA_TASK_STATE;

struct irda_task;
typedef int (*IRDA_TASK_CALLBACK) (struct irda_task *task);

struct irda_task {
	irda_queue_t q;
	magic_t magic;

	IRDA_TASK_STATE state;
	IRDA_TASK_CALLBACK function;
	IRDA_TASK_CALLBACK finished;

	struct irda_task *parent;
	struct timer_list timer;

	void *instance; /* Instance being called */
	void *param;    /* Parameter to be used by instance */
};

/* Dongle info */
struct dongle_reg;
typedef struct {
	struct dongle_reg *issue;     /* Registration info */
	struct net_device *dev;           /* Device we are attached to */
	struct irda_task *speed_task; /* Task handling speed change */
	struct irda_task *reset_task; /* Task handling reset */
	__u32 speed;                  /* Current speed */

	/* Callbacks to the IrDA device driver */
	int (*set_mode)(struct net_device *, int mode);
	int (*read)(struct net_device *dev, __u8 *buf, int len);
	int (*write)(struct net_device *dev, __u8 *buf, int len);
	int (*set_dtr_rts)(struct net_device *dev, int dtr, int rts);
} dongle_t;

/* Dongle registration info */
struct dongle_reg {
	irda_queue_t q;         /* Must be first */
	IRDA_DONGLE type;

	void (*open)(dongle_t *dongle, struct qos_info *qos);
	void (*close)(dongle_t *dongle);
	int  (*reset)(struct irda_task *task);
	int  (*change_speed)(struct irda_task *task);
	struct module *owner;
};

struct irda_skb_cb {
	unsigned int default_qdisc_pad;
	magic_t magic;       /* Be sure that we can trust the information */
	__u32   next_speed;  /* The Speed to be set *after* this frame */
	__u16   mtt;         /* Minimum turn around time */
	__u16   xbofs;       /* Number of xbofs required, used by SIR mode */
	__u16   next_xbofs;  /* Number of xbofs required *after* this frame */
	void    *context;    /* May be used by drivers */
	void    (*destructor)(struct sk_buff *skb); /* Used for flow control */
	__u16   xbofs_delay; /* Number of xbofs used for generating the mtt */
	__u8    line;        /* Used by IrCOMM in IrLPT mode */
};

/* Chip specific info */
typedef struct {
	int cfg_base;         /* Config register IO base */
        int sir_base;         /* SIR IO base */
	int fir_base;         /* FIR IO base */
	int mem_base;         /* Shared memory base */
        int sir_ext;          /* Length of SIR iobase */
	int fir_ext;          /* Length of FIR iobase */
        int irq, irq2;        /* Interrupts used */
        int dma, dma2;        /* DMA channel(s) used */
        int fifo_size;        /* FIFO size */
        int irqflags;         /* interrupt flags (ie, IRQF_SHARED|IRQF_DISABLED) */
	int direction;        /* Link direction, used by some FIR drivers */
	int enabled;          /* Powered on? */
	int suspended;        /* Suspended by APM */
	__u32 speed;          /* Currently used speed */
	__u32 new_speed;      /* Speed we must change to when Tx is finished */
	int dongle_id;        /* Dongle or transceiver currently used */
} chipio_t;

/* IO buffer specific info (inspired by struct sk_buff) */
typedef struct {
	int state;            /* Receiving state (transmit state not used) */
	int in_frame;         /* True if receiving frame */

	__u8 *head;	      /* start of buffer */
	__u8 *data;	      /* start of data in buffer */

	int len;	      /* current length of data */
	int truesize;	      /* total allocated size of buffer */
	__u16 fcs;

	struct sk_buff *skb;	/* ZeroCopy Rx in async_unwrap_char() */
} iobuff_t;

#define IRDA_SKB_MAX_MTU	2064
#define IRDA_SIR_MAX_FRAME	4269

#define IRDA_RX_COPY_THRESHOLD  256

/* Function prototypes */
int  irda_device_init(void);
void irda_device_cleanup(void);

struct irlap_cb *irlap_open(struct net_device *dev, struct qos_info *qos,
			    const char *hw_name);
void irlap_close(struct irlap_cb *self);

/* Interface to be uses by IrLAP */
void irda_device_set_media_busy(struct net_device *dev, int status);
int  irda_device_is_media_busy(struct net_device *dev);
int  irda_device_is_receiving(struct net_device *dev);

/* Interface for internal use */
static inline int irda_device_txqueue_empty(const struct net_device *dev)
{
	return qdisc_all_tx_empty(dev);
}
int  irda_device_set_raw_mode(struct net_device* self, int status);
struct net_device *alloc_irdadev(int sizeof_priv);

void irda_setup_dma(int channel, dma_addr_t buffer, int count, int mode);

static inline __u16 irda_get_mtt(const struct sk_buff *skb)
{
	const struct irda_skb_cb *cb = (const struct irda_skb_cb *) skb->cb;
	return (cb->magic == LAP_MAGIC) ? cb->mtt : 10000;
}

static inline __u32 irda_get_next_speed(const struct sk_buff *skb)
{
	const struct irda_skb_cb *cb = (const struct irda_skb_cb *) skb->cb;
	return (cb->magic == LAP_MAGIC) ? cb->next_speed : -1;
}

static inline __u16 irda_get_xbofs(const struct sk_buff *skb)
{
	const struct irda_skb_cb *cb = (const struct irda_skb_cb *) skb->cb;
	return (cb->magic == LAP_MAGIC) ? cb->xbofs : 10;
}

static inline __u16 irda_get_next_xbofs(const struct sk_buff *skb)
{
	const struct irda_skb_cb *cb = (const struct irda_skb_cb *) skb->cb;
	return (cb->magic == LAP_MAGIC) ? cb->next_xbofs : -1;
}
#endif /* IRDA_DEVICE_H */


