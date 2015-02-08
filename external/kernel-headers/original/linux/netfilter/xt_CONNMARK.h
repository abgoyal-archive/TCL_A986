#ifndef _XT_CONNMARK_H_target
#define _XT_CONNMARK_H_target


enum {
	XT_CONNMARK_SET = 0,
	XT_CONNMARK_SAVE,
	XT_CONNMARK_RESTORE
};

struct xt_connmark_target_info {
	unsigned long mark;
	unsigned long mask;
	u_int8_t mode;
};

#endif /*_XT_CONNMARK_H_target*/
