
#ifndef _IPT_CONNMARK_H
#define _IPT_CONNMARK_H


struct ipt_connmark_info {
#ifdef KERNEL_64_USERSPACE_32
	unsigned long long mark, mask;
#else
	unsigned long mark, mask;
#endif
	u_int8_t invert;
};

#endif /*_IPT_CONNMARK_H*/
