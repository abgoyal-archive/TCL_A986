
#ifndef __debug_levels__h__
#define __debug_levels__h__

/* Maximum compile and run time debug level for all submodules */
#define D_MODULENAME wimax
#define D_MASTER CONFIG_WIMAX_DEBUG_LEVEL

#include <linux/wimax/debug.h>

/* List of all the enabled modules */
enum d_module {
	D_SUBMODULE_DECLARE(debugfs),
	D_SUBMODULE_DECLARE(id_table),
	D_SUBMODULE_DECLARE(op_msg),
	D_SUBMODULE_DECLARE(op_reset),
	D_SUBMODULE_DECLARE(op_rfkill),
	D_SUBMODULE_DECLARE(op_state_get),
	D_SUBMODULE_DECLARE(stack),
};

#endif /* #ifndef __debug_levels__h__ */
