#ifndef _XT_RATE_H
#define _XT_RATE_H

/* timings are in milliseconds. */
#define XT_LIMIT_SCALE 10000

struct xt_rateinfo {
	u_int32_t avg;    /* Average secs between packets * scale */
	u_int32_t burst;  /* Period multiplier for upper limit. */

	/* Used internally by the kernel */
	unsigned long prev;
	u_int32_t credit;
	u_int32_t credit_cap, cost;

	/* Ugly, ugly fucker. */
	struct xt_rateinfo *master;
};
#endif /*_XT_RATE_H*/
