

#include "clock.h"

/* clksel_rate data common to 24xx/343x */
const struct clksel_rate gpt_32k_rates[] = {
	 { .div = 1, .val = 0, .flags = RATE_IN_24XX | RATE_IN_3XXX },
	 { .div = 0 }
};

const struct clksel_rate gpt_sys_rates[] = {
	 { .div = 1, .val = 1, .flags = RATE_IN_24XX | RATE_IN_3XXX },
	 { .div = 0 }
};

const struct clksel_rate gfx_l3_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_24XX | RATE_IN_3XXX },
	{ .div = 2, .val = 2, .flags = RATE_IN_24XX | RATE_IN_3XXX },
	{ .div = 3, .val = 3, .flags = RATE_IN_243X | RATE_IN_3XXX },
	{ .div = 4, .val = 4, .flags = RATE_IN_243X | RATE_IN_3XXX },
	{ .div = 0 }
};

