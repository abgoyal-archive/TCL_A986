



#ifndef __BFAD_TM_H__
#define __BFAD_TM_H__

#include <defs/bfa_defs_status.h>

#define FCPT_NAME 		""


/* attach tgt template with scst */
#define bfad_tm_module_init()	do {} while (0)

/* detach/release tgt template */
#define bfad_tm_module_exit()	do {} while (0)

#define bfad_tm_probe(x)	do {} while (0)
#define bfad_tm_probe_undo(x)	do {} while (0)
#define bfad_tm_probe_post(x)	do {} while (0)

#define bfad_tm_port_new(x, y)		BFA_STATUS_OK
#define bfad_tm_port_delete(x, y)	do {} while (0)

#define bfad_tm_port_online(x, y)	do {} while (0)
#define bfad_tm_port_offline(x, y)	do {} while (0)

#endif
