

#ifndef _LINUX_IF_EQL_H
#define _LINUX_IF_EQL_H

#define EQL_DEFAULT_SLAVE_PRIORITY 28800
#define EQL_DEFAULT_MAX_SLAVES     4
#define EQL_DEFAULT_MTU            576
#define EQL_DEFAULT_RESCHED_IVAL   100

#define EQL_ENSLAVE     (SIOCDEVPRIVATE)
#define EQL_EMANCIPATE  (SIOCDEVPRIVATE + 1)

#define EQL_GETSLAVECFG (SIOCDEVPRIVATE + 2)
#define EQL_SETSLAVECFG (SIOCDEVPRIVATE + 3)

#define EQL_GETMASTRCFG (SIOCDEVPRIVATE + 4)
#define EQL_SETMASTRCFG (SIOCDEVPRIVATE + 5)

#ifdef __KERNEL__

#include <linux/timer.h>
#include <linux/spinlock.h>

typedef struct slave {
	struct list_head	list;
	struct net_device	*dev;
	long			priority;
	long			priority_bps;
	long			priority_Bps;
	long			bytes_queued;
} slave_t;

typedef struct slave_queue {
	spinlock_t		lock;
	struct list_head	all_slaves;
	int			num_slaves;
	struct net_device	*master_dev;
} slave_queue_t;

typedef struct equalizer {
	slave_queue_t		queue;
	int			min_slaves;
	int			max_slaves;
	struct timer_list	timer;
} equalizer_t;  

#endif /* __KERNEL__ */

typedef struct master_config {
	char	master_name[16];
	int	max_slaves;
	int	min_slaves;
} master_config_t;

typedef struct slave_config {
	char	slave_name[16];
	long	priority;
} slave_config_t;

typedef struct slaving_request {
	char	slave_name[16];
	long	priority;
} slaving_request_t;


#endif /* _LINUX_EQL_H */
