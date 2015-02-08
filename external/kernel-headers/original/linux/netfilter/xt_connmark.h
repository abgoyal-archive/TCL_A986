#ifndef _XT_CONNMARK_H
#define _XT_CONNMARK_H


struct xt_connmark_info {
	unsigned long mark, mask;
	u_int8_t invert;
};

#endif /*_XT_CONNMARK_H*/
