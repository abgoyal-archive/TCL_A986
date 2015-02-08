

#ifndef __NET_TC_SKBEDIT_H
#define __NET_TC_SKBEDIT_H

#include <net/act_api.h>

struct tcf_skbedit {
	struct tcf_common	common;
	u32			flags;
	u32     		priority;
	u32     		mark;
	u16			queue_mapping;
	/* XXX: 16-bit pad here? */
};
#define to_skbedit(pc) \
	container_of(pc, struct tcf_skbedit, common)

#endif /* __NET_TC_SKBEDIT_H */
