


#ifndef _NETLABEL_USER_H
#define _NETLABEL_USER_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/capability.h>
#include <linux/audit.h>
#include <net/netlink.h>
#include <net/genetlink.h>
#include <net/netlabel.h>

/* NetLabel NETLINK helper functions */

static inline void netlbl_netlink_auditinfo(struct sk_buff *skb,
					    struct netlbl_audit *audit_info)
{
	audit_info->secid = NETLINK_CB(skb).sid;
	audit_info->loginuid = NETLINK_CB(skb).loginuid;
	audit_info->sessionid = NETLINK_CB(skb).sessionid;
}

/* NetLabel NETLINK I/O functions */

int netlbl_netlink_init(void);

/* NetLabel Audit Functions */

struct audit_buffer *netlbl_audit_start_common(int type,
					      struct netlbl_audit *audit_info);

#endif
