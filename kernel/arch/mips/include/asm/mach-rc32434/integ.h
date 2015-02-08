

#ifndef __RC32434_INTEG_H__
#define __RC32434_INTEG_H__

#include <asm/mach-rc32434/rb.h>

#define INTEG0_BASE_ADDR	0x18030030

struct integ {
	u32 errcs;			/* sticky use ERRCS_ */
	u32 wtcount;			/* Watchdog timer count reg. */
	u32 wtcompare;			/* Watchdog timer timeout value. */
	u32 wtc;			/* Watchdog timer control. use WTC_ */
};

/* Error counters */
#define RC32434_ERR_WTO		0
#define RC32434_ERR_WNE		1
#define RC32434_ERR_UCW		2
#define RC32434_ERR_UCR		3
#define RC32434_ERR_UPW		4
#define RC32434_ERR_UPR		5
#define RC32434_ERR_UDW		6
#define RC32434_ERR_UDR		7
#define RC32434_ERR_SAE		8
#define RC32434_ERR_WRE		9

/* Watchdog control bits */
#define RC32434_WTC_EN		0
#define RC32434_WTC_TO		1

#endif	/* __RC32434_INTEG_H__ */
