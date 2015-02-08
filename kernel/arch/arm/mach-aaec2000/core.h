

#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>

struct sys_timer;

extern struct sys_timer aaec2000_timer;
extern void __init aaec2000_map_io(void);
extern void __init aaec2000_init_irq(void);

struct aaec2000_clcd_info {
	struct clcd_panel panel;
	void (*disable)(struct clcd_fb *);
	void (*enable)(struct clcd_fb *);
};

extern void __init aaec2000_set_clcd_plat_data(struct aaec2000_clcd_info *);

