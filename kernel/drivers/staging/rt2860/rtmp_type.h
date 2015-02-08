
#ifndef __RTMP_TYPE_H__
#define __RTMP_TYPE_H__

#include <linux/types.h>

#define PACKED  __attribute__ ((packed))

typedef unsigned char BOOLEAN;

typedef union _LARGE_INTEGER {
	struct {
		u32 LowPart;
		int HighPart;
	} u;
	long long QuadPart;
} LARGE_INTEGER;

/* */
/* Register set pair for initialzation register set definition */
/* */
struct rt_rtmp_reg_pair {
	unsigned long Register;
	unsigned long Value;
};

struct rt_reg_pair {
	u8 Register;
	u8 Value;
};

/* */
/* Register set pair for initialzation register set definition */
/* */
struct rt_rtmp_rf_regs {
	u8 Channel;
	unsigned long R1;
	unsigned long R2;
	unsigned long R3;
	unsigned long R4;
};

struct rt_frequency_item {
	u8 Channel;
	u8 N;
	u8 R;
	u8 K;
};

#define STATUS_SUCCESS				0x00
#define STATUS_UNSUCCESSFUL		0x01

#endif /* __RTMP_TYPE_H__ // */
