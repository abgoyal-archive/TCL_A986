

#ifndef __BFA_DEFS_AUDIT_H__
#define __BFA_DEFS_AUDIT_H__

#include <bfa_os_inc.h>

enum bfa_audit_aen_event {
	BFA_AUDIT_AEN_AUTH_ENABLE 	= 1,
	BFA_AUDIT_AEN_AUTH_DISABLE 	= 2,
};

struct bfa_audit_aen_data_s {
	wwn_t           pwwn;
};

#endif /* __BFA_DEFS_AUDIT_H__ */
