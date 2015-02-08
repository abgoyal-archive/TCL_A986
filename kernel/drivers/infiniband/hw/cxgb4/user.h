
#ifndef __C4IW_USER_H__
#define __C4IW_USER_H__

#define C4IW_UVERBS_ABI_VERSION	1

struct c4iw_create_cq_resp {
	__u64 key;
	__u64 gts_key;
	__u64 memsize;
	__u32 cqid;
	__u32 size;
	__u32 qid_mask;
};

struct c4iw_create_qp_resp {
	__u64 sq_key;
	__u64 rq_key;
	__u64 sq_db_gts_key;
	__u64 rq_db_gts_key;
	__u64 sq_memsize;
	__u64 rq_memsize;
	__u32 sqid;
	__u32 rqid;
	__u32 sq_size;
	__u32 rq_size;
	__u32 qid_mask;
};
#endif
