

#ifndef __W1_NETLINK_H
#define __W1_NETLINK_H

#include <asm/types.h>
#include <linux/connector.h>

#include "w1.h"

enum w1_netlink_message_types {
	W1_SLAVE_ADD = 0,
	W1_SLAVE_REMOVE,
	W1_MASTER_ADD,
	W1_MASTER_REMOVE,
	W1_MASTER_CMD,
	W1_SLAVE_CMD,
	W1_LIST_MASTERS,
};

struct w1_netlink_msg
{
	__u8				type;
	__u8				status;
	__u16				len;
	union {
		__u8			id[8];
		struct w1_mst {
			__u32		id;
			__u32		res;
		} mst;
	} id;
	__u8				data[0];
};

enum w1_commands {
	W1_CMD_READ = 0,
	W1_CMD_WRITE,
	W1_CMD_SEARCH,
	W1_CMD_ALARM_SEARCH,
	W1_CMD_TOUCH,
	W1_CMD_RESET,
	W1_CMD_MAX,
};

struct w1_netlink_cmd
{
	__u8				cmd;
	__u8				res;
	__u16				len;
	__u8				data[0];
};

#ifdef __KERNEL__

void w1_netlink_send(struct w1_master *, struct w1_netlink_msg *);
int w1_init_netlink(void);
void w1_fini_netlink(void);

#endif /* __KERNEL__ */
#endif /* __W1_NETLINK_H */
