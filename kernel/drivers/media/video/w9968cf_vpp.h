

#ifndef _W9968CF_VPP_H_
#define _W9968CF_VPP_H_

#include <linux/module.h>
#include <asm/types.h>

struct w9968cf_vpp_t {
	struct module* owner;
	int (*check_headers)(const unsigned char*, const unsigned long);
	int (*decode)(const char*, const unsigned long, const unsigned,
		      const unsigned, char*);
	void (*swap_yuvbytes)(void*, unsigned long);
	void (*uyvy_to_rgbx)(u8*, unsigned long, u8*, u16, u8);
	void (*scale_up)(u8*, u8*, u16, u16, u16, u16, u16);

	u8 busy; /* read-only flag: module is/is not in use */
};

#endif /* _W9968CF_VPP_H_ */
