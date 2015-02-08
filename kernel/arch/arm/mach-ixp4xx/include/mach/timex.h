

#include <mach/hardware.h>

#define FREQ 66666000
#define CLOCK_TICK_RATE (((FREQ / HZ & ~IXP4XX_OST_RELOAD_MASK) + 1) * HZ)

