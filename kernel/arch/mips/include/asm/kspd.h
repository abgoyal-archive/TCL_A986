

#ifndef _ASM_KSPD_H
#define _ASM_KSPD_H

struct kspd_notifications {
	void (*kspd_sp_exit)(int sp_id);

	struct list_head list;
};

#ifdef CONFIG_MIPS_APSP_KSPD
extern void kspd_notify(struct kspd_notifications *notify);
#else
static inline void kspd_notify(struct kspd_notifications *notify)
{
}
#endif

#endif
