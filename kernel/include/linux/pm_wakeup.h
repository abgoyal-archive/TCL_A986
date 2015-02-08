

#ifndef _LINUX_PM_WAKEUP_H
#define _LINUX_PM_WAKEUP_H

#ifndef _DEVICE_H_
# error "please don't include this file directly"
#endif

#include <linux/types.h>

#ifdef CONFIG_PM

static inline void device_init_wakeup(struct device *dev, bool val)
{
	dev->power.can_wakeup = dev->power.should_wakeup = val;
}

static inline void device_set_wakeup_capable(struct device *dev, bool capable)
{
	dev->power.can_wakeup = capable;
}

static inline bool device_can_wakeup(struct device *dev)
{
	return dev->power.can_wakeup;
}

static inline void device_set_wakeup_enable(struct device *dev, bool enable)
{
	dev->power.should_wakeup = enable;
}

static inline bool device_may_wakeup(struct device *dev)
{
	return dev->power.can_wakeup && dev->power.should_wakeup;
}

#else /* !CONFIG_PM */

/* For some reason the next two routines work even without CONFIG_PM */
static inline void device_init_wakeup(struct device *dev, bool val)
{
	dev->power.can_wakeup = val;
}

static inline void device_set_wakeup_capable(struct device *dev, bool capable)
{
}

static inline bool device_can_wakeup(struct device *dev)
{
	return dev->power.can_wakeup;
}

static inline void device_set_wakeup_enable(struct device *dev, bool enable)
{
}

static inline bool device_may_wakeup(struct device *dev)
{
	return false;
}

#endif /* !CONFIG_PM */

#endif /* _LINUX_PM_WAKEUP_H */
