
#include <linux/init.h>

extern void sb1250_clocksource_init(void);
extern void sb1250_clockevent_init(void);

void __init plat_time_init(void)
{
	sb1250_clocksource_init();
	sb1250_clockevent_init();
}
