
#ifndef _XT_NFQ_TARGET_H
#define _XT_NFQ_TARGET_H

#include <linux/types.h>

/* target info */
struct xt_NFQ_info {
	__u16 queuenum;
};

struct xt_NFQ_info_v1 {
	__u16 queuenum;
	__u16 queues_total;
};

#endif /* _XT_NFQ_TARGET_H */
