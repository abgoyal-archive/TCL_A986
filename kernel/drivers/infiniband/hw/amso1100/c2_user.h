

#ifndef C2_USER_H
#define C2_USER_H

#include <linux/types.h>


struct c2_alloc_ucontext_resp {
	__u32 qp_tab_size;
	__u32 uarc_size;
};

struct c2_alloc_pd_resp {
	__u32 pdn;
	__u32 reserved;
};

struct c2_create_cq {
	__u32 lkey;
	__u32 pdn;
	__u64 arm_db_page;
	__u64 set_db_page;
	__u32 arm_db_index;
	__u32 set_db_index;
};

struct c2_create_cq_resp {
	__u32 cqn;
	__u32 reserved;
};

struct c2_create_qp {
	__u32 lkey;
	__u32 reserved;
	__u64 sq_db_page;
	__u64 rq_db_page;
	__u32 sq_db_index;
	__u32 rq_db_index;
};

#endif				/* C2_USER_H */
