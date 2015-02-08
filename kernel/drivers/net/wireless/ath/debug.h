

#ifndef ATH_DEBUG_H
#define ATH_DEBUG_H

#include "ath.h"

enum ATH_DEBUG {
	ATH_DBG_RESET		= 0x00000001,
	ATH_DBG_QUEUE		= 0x00000002,
	ATH_DBG_EEPROM		= 0x00000004,
	ATH_DBG_CALIBRATE	= 0x00000008,
	ATH_DBG_INTERRUPT	= 0x00000010,
	ATH_DBG_REGULATORY	= 0x00000020,
	ATH_DBG_ANI		= 0x00000040,
	ATH_DBG_XMIT		= 0x00000080,
	ATH_DBG_BEACON		= 0x00000100,
	ATH_DBG_CONFIG		= 0x00000200,
	ATH_DBG_FATAL		= 0x00000400,
	ATH_DBG_PS		= 0x00000800,
	ATH_DBG_HWTIMER		= 0x00001000,
	ATH_DBG_BTCOEX		= 0x00002000,
	ATH_DBG_WMI		= 0x00004000,
	ATH_DBG_ANY		= 0xffffffff
};

#define ATH_DBG_DEFAULT (ATH_DBG_FATAL)

#ifdef CONFIG_ATH_DEBUG
void ath_print(struct ath_common *common, int dbg_mask, const char *fmt, ...)
	__attribute__ ((format (printf, 3, 4)));
#else
static inline void __attribute__ ((format (printf, 3, 4)))
ath_print(struct ath_common *common, int dbg_mask, const char *fmt, ...)
{
}
#endif /* CONFIG_ATH_DEBUG */

#endif /* ATH_DEBUG_H */
