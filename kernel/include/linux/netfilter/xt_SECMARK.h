
#ifndef _XT_SECMARK_H_target
#define _XT_SECMARK_H_target

#include <linux/types.h>

#define SECMARK_MODE_SEL	0x01		/* SELinux */
#define SECMARK_SELCTX_MAX	256

struct xt_secmark_target_selinux_info {
	__u32 selsid;
	char selctx[SECMARK_SELCTX_MAX];
};

struct xt_secmark_target_info {
	__u8 mode;
	union {
		struct xt_secmark_target_selinux_info sel;
	} u;
};

#endif /*_XT_SECMARK_H_target */
