

#ifndef __LINUX__WIMAX_H__
#define __LINUX__WIMAX_H__

#include <linux/types.h>

enum {
	/**
	 * Version of the interface (unsigned decimal, MMm, max 25.5)
	 * M - Major: change if removing or modifying an existing call.
	 * m - minor: change when adding a new call
	 */
	WIMAX_GNL_VERSION = 01,
	/* Generic NetLink attributes */
	WIMAX_GNL_ATTR_INVALID = 0x00,
	WIMAX_GNL_ATTR_MAX = 10,
};


enum {
	WIMAX_GNL_OP_MSG_FROM_USER,	/* User to kernel message */
	WIMAX_GNL_OP_MSG_TO_USER,	/* Kernel to user message */
	WIMAX_GNL_OP_RFKILL,	/* Run wimax_rfkill() */
	WIMAX_GNL_OP_RESET,	/* Run wimax_rfkill() */
	WIMAX_GNL_RE_STATE_CHANGE,	/* Report: status change */
	WIMAX_GNL_OP_STATE_GET,		/* Request for current state */
};


/* Message from user / to user */
enum {
	WIMAX_GNL_MSG_IFIDX = 1,
	WIMAX_GNL_MSG_PIPE_NAME,
	WIMAX_GNL_MSG_DATA,
};


enum wimax_rf_state {
	WIMAX_RF_OFF = 0,	/* Radio is off, rfkill on/enabled */
	WIMAX_RF_ON = 1,	/* Radio is on, rfkill off/disabled */
	WIMAX_RF_QUERY = 2,
};

/* Attributes */
enum {
	WIMAX_GNL_RFKILL_IFIDX = 1,
	WIMAX_GNL_RFKILL_STATE,
};


/* Attributes for wimax_reset() */
enum {
	WIMAX_GNL_RESET_IFIDX = 1,
};

/* Atributes for wimax_state_get() */
enum {
	WIMAX_GNL_STGET_IFIDX = 1,
};

enum {
	WIMAX_GNL_STCH_IFIDX = 1,
	WIMAX_GNL_STCH_STATE_OLD,
	WIMAX_GNL_STCH_STATE_NEW,
};


 enum wimax_st {
	__WIMAX_ST_NULL = 0,
	WIMAX_ST_DOWN,
	__WIMAX_ST_QUIESCING,
	WIMAX_ST_UNINITIALIZED,
	WIMAX_ST_RADIO_OFF,
	WIMAX_ST_READY,
	WIMAX_ST_SCANNING,
	WIMAX_ST_CONNECTING,
	WIMAX_ST_CONNECTED,
	__WIMAX_ST_INVALID			/* Always keep last */
};


#endif /* #ifndef __LINUX__WIMAX_H__ */
