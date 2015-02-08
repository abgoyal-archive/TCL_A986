
#ifndef __BFA_DEFS_POM_H__
#define __BFA_DEFS_POM_H__

#include <bfa_os_inc.h>
#include <defs/bfa_defs_types.h>

enum bfa_pom_entry_health {
	BFA_POM_HEALTH_NOINFO  = 1,	/*  no information */
	BFA_POM_HEALTH_NORMAL  = 2,	/*  health is normal */
	BFA_POM_HEALTH_WARNING = 3,	/*  warning level */
	BFA_POM_HEALTH_ALARM   = 4,	/*  alarming level */
};

struct bfa_pom_entry_s {
	enum bfa_pom_entry_health health;	/*  POM entry health */
	u32        curr_value;	/*  current value */
	u32        thr_warn_high;	/*  threshold warning high */
	u32        thr_warn_low;	/*  threshold warning low */
	u32        thr_alarm_low;	/*  threshold alaram low */
	u32        thr_alarm_high;	/*  threshold alarm high */
};

struct bfa_pom_attr_s {
	struct bfa_pom_entry_s temperature;	/*  centigrade */
	struct bfa_pom_entry_s voltage;	/*  volts */
	struct bfa_pom_entry_s curr;	/*  milli amps */
	struct bfa_pom_entry_s txpower;	/*  micro watts */
	struct bfa_pom_entry_s rxpower;	/*  micro watts */
};

#endif /* __BFA_DEFS_POM_H__ */
