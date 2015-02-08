

#ifndef _LINUX_MQUEUE_H
#define _LINUX_MQUEUE_H

#define MQ_PRIO_MAX 	32768
/* per-uid limit of kernel memory used by mqueue, in bytes */
#define MQ_BYTES_MAX	819200

struct mq_attr {
	long	mq_flags;	/* message queue flags			*/
	long	mq_maxmsg;	/* maximum number of messages		*/
	long	mq_msgsize;	/* maximum message size			*/
	long	mq_curmsgs;	/* number of messages currently queued	*/
	long	__reserved[4];	/* ignored for input, zeroed for output */
};

#define NOTIFY_NONE	0
#define NOTIFY_WOKENUP	1
#define NOTIFY_REMOVED	2

#define NOTIFY_COOKIE_LEN	32

#endif
