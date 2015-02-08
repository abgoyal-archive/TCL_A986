

#ifndef __BFA_DEFS_LPORT_H__
#define __BFA_DEFS_LPORT_H__

#include <defs/bfa_defs_types.h>
#include <defs/bfa_defs_port.h>

enum bfa_lport_aen_event {
	BFA_LPORT_AEN_NEW	= 1,	/*  LPort created event */
	BFA_LPORT_AEN_DELETE	= 2,	/*  LPort deleted event */
	BFA_LPORT_AEN_ONLINE	= 3,	/*  LPort online event */
	BFA_LPORT_AEN_OFFLINE	= 4,	/*  LPort offline event */
	BFA_LPORT_AEN_DISCONNECT = 5,	/*  LPort disconnect event */
	BFA_LPORT_AEN_NEW_PROP	= 6,	/*  VPort created event */
	BFA_LPORT_AEN_DELETE_PROP = 7,	/*  VPort deleted event */
	BFA_LPORT_AEN_NEW_STANDARD = 8,	/*  VPort created event */
	BFA_LPORT_AEN_DELETE_STANDARD = 9,  /*  VPort deleted event */
	BFA_LPORT_AEN_NPIV_DUP_WWN = 10,    /*  VPort configured with
					     *   duplicate WWN event
						 */
	BFA_LPORT_AEN_NPIV_FABRIC_MAX = 11, /*  Max NPIV in fabric/fport */
	BFA_LPORT_AEN_NPIV_UNKNOWN = 12, /*  Unknown NPIV Error code event */
};

struct bfa_lport_aen_data_s {
	u16        vf_id;	/*  vf_id of this logical port */
	s16         roles;  /*  Logical port mode,IM/TM/IP etc */
	u32        rsvd;
	wwn_t           ppwwn;	/*  WWN of its physical port */
	wwn_t           lpwwn;	/*  WWN of this logical port */
};

#endif /* __BFA_DEFS_LPORT_H__ */
