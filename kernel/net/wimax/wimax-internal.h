

#ifndef __WIMAX_INTERNAL_H__
#define __WIMAX_INTERNAL_H__
#ifdef __KERNEL__

#include <linux/device.h>
#include <net/wimax.h>


static inline __must_check
int wimax_dev_is_ready(struct wimax_dev *wimax_dev)
{
	if (wimax_dev->state == __WIMAX_ST_NULL)
		return -EINVAL;	/* Device is not even registered! */
	if (wimax_dev->state == WIMAX_ST_DOWN)
		return -ENOMEDIUM;
	if (wimax_dev->state == __WIMAX_ST_QUIESCING)
		return -ESHUTDOWN;
	return 0;
}


static inline
void __wimax_state_set(struct wimax_dev *wimax_dev, enum wimax_st state)
{
	wimax_dev->state = state;
}
extern void __wimax_state_change(struct wimax_dev *, enum wimax_st);

#ifdef CONFIG_DEBUG_FS
extern int wimax_debugfs_add(struct wimax_dev *);
extern void wimax_debugfs_rm(struct wimax_dev *);
#else
static inline int wimax_debugfs_add(struct wimax_dev *wimax_dev)
{
	return 0;
}
static inline void wimax_debugfs_rm(struct wimax_dev *wimax_dev) {}
#endif

extern void wimax_id_table_add(struct wimax_dev *);
extern struct wimax_dev *wimax_dev_get_by_genl_info(struct genl_info *, int);
extern void wimax_id_table_rm(struct wimax_dev *);
extern void wimax_id_table_release(void);

extern int wimax_rfkill_add(struct wimax_dev *);
extern void wimax_rfkill_rm(struct wimax_dev *);

extern struct genl_family wimax_gnl_family;
extern struct genl_multicast_group wimax_gnl_mcg;

#endif /* #ifdef __KERNEL__ */
#endif /* #ifndef __WIMAX_INTERNAL_H__ */
