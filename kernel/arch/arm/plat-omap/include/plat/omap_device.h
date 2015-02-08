
#ifndef __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_DEVICE_H
#define __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_DEVICE_H

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <plat/omap_hwmod.h>

/* omap_device._state values */
#define OMAP_DEVICE_STATE_UNKNOWN	0
#define OMAP_DEVICE_STATE_ENABLED	1
#define OMAP_DEVICE_STATE_IDLE		2
#define OMAP_DEVICE_STATE_SHUTDOWN	3

struct omap_device {
	u32                             magic;
	struct platform_device		pdev;
	struct omap_hwmod		**hwmods;
	struct omap_device_pm_latency	*pm_lats;
	u32				dev_wakeup_lat;
	u32				_dev_wakeup_lat_limit;
	u8				pm_lats_cnt;
	s8				pm_lat_level;
	u8				hwmods_cnt;
	u8				_state;
};

/* Device driver interface (call via platform_data fn ptrs) */

int omap_device_enable(struct platform_device *pdev);
int omap_device_idle(struct platform_device *pdev);
int omap_device_shutdown(struct platform_device *pdev);

/* Core code interface */

bool omap_device_is_valid(struct omap_device *od);
int omap_device_count_resources(struct omap_device *od);
int omap_device_fill_resources(struct omap_device *od, struct resource *res);

struct omap_device *omap_device_build(const char *pdev_name, int pdev_id,
				      struct omap_hwmod *oh, void *pdata,
				      int pdata_len,
				      struct omap_device_pm_latency *pm_lats,
				      int pm_lats_cnt, int is_early_device);

struct omap_device *omap_device_build_ss(const char *pdev_name, int pdev_id,
					 struct omap_hwmod **oh, int oh_cnt,
					 void *pdata, int pdata_len,
					 struct omap_device_pm_latency *pm_lats,
					 int pm_lats_cnt, int is_early_device);

int omap_device_register(struct omap_device *od);
int omap_early_device_register(struct omap_device *od);

/* OMAP PM interface */
int omap_device_align_pm_lat(struct platform_device *pdev,
			     u32 new_wakeup_lat_limit);
struct powerdomain *omap_device_get_pwrdm(struct omap_device *od);

/* Other */

int omap_device_idle_hwmods(struct omap_device *od);
int omap_device_enable_hwmods(struct omap_device *od);

int omap_device_disable_clocks(struct omap_device *od);
int omap_device_enable_clocks(struct omap_device *od);


struct omap_device_pm_latency {
	u32 deactivate_lat;
	u32 deactivate_lat_worst;
	int (*deactivate_func)(struct omap_device *od);
	u32 activate_lat;
	u32 activate_lat_worst;
	int (*activate_func)(struct omap_device *od);
	u32 flags;
};

#define OMAP_DEVICE_LATENCY_AUTO_ADJUST BIT(1)

/* Get omap_device pointer from platform_device pointer */
#define to_omap_device(x) container_of((x), struct omap_device, pdev)

#endif
