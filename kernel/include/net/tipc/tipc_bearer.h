

#ifndef _NET_TIPC_BEARER_H_
#define _NET_TIPC_BEARER_H_

#ifdef __KERNEL__

#include <linux/tipc_config.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>


#define TIPC_MEDIA_TYPE_ETH	1


struct tipc_media_addr {
	__be32  type;			/* bearer type (network byte order) */
	union {
		__u8   eth_addr[6];	/* 48 bit Ethernet addr (byte array) */ 
#if 0
		/* Prototypes for other possible bearer types */

		struct {
			__u16 sin_family;
			__u16 sin_port;
			struct {
				__u32 s_addr;
			} sin_addr;
			char pad[4];
		} addr_in;		/* IP-based bearer */
		__u16  sock_descr;	/* generic socket bearer */
#endif
	} dev_addr;
};


struct tipc_bearer {
	void *usr_handle;
	u32 mtu;
	int blocked;
	spinlock_t lock;
	struct tipc_media_addr addr;
	char name[TIPC_MAX_BEARER_NAME];
};


int  tipc_register_media(u32 media_type,
			 char *media_name, 
			 int (*enable)(struct tipc_bearer *), 
			 void (*disable)(struct tipc_bearer *), 
			 int (*send_msg)(struct sk_buff *, 
					 struct tipc_bearer *,
					 struct tipc_media_addr *), 
			 char *(*addr2str)(struct tipc_media_addr *a,
					   char *str_buf,
					   int str_size),
			 struct tipc_media_addr *bcast_addr,
			 const u32 bearer_priority,
			 const u32 link_tolerance,  /* [ms] */
			 const u32 send_window_limit); 

void tipc_recv_msg(struct sk_buff *buf, struct tipc_bearer *tb_ptr);

int  tipc_block_bearer(const char *name);
void tipc_continue(struct tipc_bearer *tb_ptr); 

int tipc_enable_bearer(const char *bearer_name, u32 bcast_scope, u32 priority);
int tipc_disable_bearer(const char *name);


int  tipc_eth_media_start(void);
void tipc_eth_media_stop(void);

#endif

#endif
