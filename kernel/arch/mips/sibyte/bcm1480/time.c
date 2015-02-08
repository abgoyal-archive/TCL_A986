
#include <linux/init.h>

extern void sb1480_clockevent_init(void);
extern void sb1480_clocksource_init(void);

void __init plat_time_init(void)
{
	sb1480_clocksource_init();
	sb1480_clockevent_init();
}
