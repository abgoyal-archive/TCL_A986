


#ifndef _DRIVERS_MISC_SGIXP_XP_H
#define _DRIVERS_MISC_SGIXP_XP_H

#include <linux/mutex.h>

#if defined CONFIG_X86_UV || defined CONFIG_IA64_SGI_UV
#include <asm/uv/uv.h>
#define is_uv()		is_uv_system()
#endif

#ifndef is_uv
#define is_uv()		0
#endif

#if defined CONFIG_IA64
#include <asm/system.h>
#include <asm/sn/arch.h>	/* defines is_shub1() and is_shub2() */
#define is_shub()	ia64_platform_is("sn2")
#endif

#ifndef is_shub1
#define is_shub1()	0
#endif

#ifndef is_shub2
#define is_shub2()	0
#endif

#ifndef is_shub
#define is_shub()	0
#endif

#ifdef USE_DBUG_ON
#define DBUG_ON(condition)	BUG_ON(condition)
#else
#define DBUG_ON(condition)
#endif

#define XP_MAX_NPARTITIONS_SN2	64
#define XP_MAX_NPARTITIONS_UV	256

#define XPC_MEM_CHANNEL		0	/* memory channel number */
#define	XPC_NET_CHANNEL		1	/* network channel number */

#define XPC_MAX_NCHANNELS	2	/* max #of channels allowed */

#if XPC_MAX_NCHANNELS > 8
#error	XPC_MAX_NCHANNELS exceeds absolute MAXIMUM possible.
#endif

#define XPC_MSG_MAX_SIZE	128
#define XPC_MSG_HDR_MAX_SIZE	16
#define XPC_MSG_PAYLOAD_MAX_SIZE (XPC_MSG_MAX_SIZE - XPC_MSG_HDR_MAX_SIZE)

#define XPC_MSG_SIZE(_payload_size) \
				ALIGN(XPC_MSG_HDR_MAX_SIZE + (_payload_size), \
				      is_uv() ? 64 : 128)


enum xp_retval {
	xpSuccess = 0,

	xpNotConnected,		/*  1: channel is not connected */
	xpConnected,		/*  2: channel connected (opened) */
	xpRETIRED1,		/*  3: (formerly xpDisconnected) */

	xpMsgReceived,		/*  4: message received */
	xpMsgDelivered,		/*  5: message delivered and acknowledged */

	xpRETIRED2,		/*  6: (formerly xpTransferFailed) */

	xpNoWait,		/*  7: operation would require wait */
	xpRetry,		/*  8: retry operation */
	xpTimeout,		/*  9: timeout in xpc_allocate_msg_wait() */
	xpInterrupted,		/* 10: interrupted wait */

	xpUnequalMsgSizes,	/* 11: message size disparity between sides */
	xpInvalidAddress,	/* 12: invalid address */

	xpNoMemory,		/* 13: no memory available for XPC structures */
	xpLackOfResources,	/* 14: insufficient resources for operation */
	xpUnregistered,		/* 15: channel is not registered */
	xpAlreadyRegistered,	/* 16: channel is already registered */

	xpPartitionDown,	/* 17: remote partition is down */
	xpNotLoaded,		/* 18: XPC module is not loaded */
	xpUnloading,		/* 19: this side is unloading XPC module */

	xpBadMagic,		/* 20: XPC MAGIC string not found */

	xpReactivating,		/* 21: remote partition was reactivated */

	xpUnregistering,	/* 22: this side is unregistering channel */
	xpOtherUnregistering,	/* 23: other side is unregistering channel */

	xpCloneKThread,		/* 24: cloning kernel thread */
	xpCloneKThreadFailed,	/* 25: cloning kernel thread failed */

	xpNoHeartbeat,		/* 26: remote partition has no heartbeat */

	xpPioReadError,		/* 27: PIO read error */
	xpPhysAddrRegFailed,	/* 28: registration of phys addr range failed */

	xpRETIRED3,		/* 29: (formerly xpBteDirectoryError) */
	xpRETIRED4,		/* 30: (formerly xpBtePoisonError) */
	xpRETIRED5,		/* 31: (formerly xpBteWriteError) */
	xpRETIRED6,		/* 32: (formerly xpBteAccessError) */
	xpRETIRED7,		/* 33: (formerly xpBtePWriteError) */
	xpRETIRED8,		/* 34: (formerly xpBtePReadError) */
	xpRETIRED9,		/* 35: (formerly xpBteTimeOutError) */
	xpRETIRED10,		/* 36: (formerly xpBteXtalkError) */
	xpRETIRED11,		/* 37: (formerly xpBteNotAvailable) */
	xpRETIRED12,		/* 38: (formerly xpBteUnmappedError) */

	xpBadVersion,		/* 39: bad version number */
	xpVarsNotSet,		/* 40: the XPC variables are not set up */
	xpNoRsvdPageAddr,	/* 41: unable to get rsvd page's phys addr */
	xpInvalidPartid,	/* 42: invalid partition ID */
	xpLocalPartid,		/* 43: local partition ID */

	xpOtherGoingDown,	/* 44: other side going down, reason unknown */
	xpSystemGoingDown,	/* 45: system is going down, reason unknown */
	xpSystemHalt,		/* 46: system is being halted */
	xpSystemReboot,		/* 47: system is being rebooted */
	xpSystemPoweroff,	/* 48: system is being powered off */

	xpDisconnecting,	/* 49: channel disconnecting (closing) */

	xpOpenCloseError,	/* 50: channel open/close protocol error */

	xpDisconnected,		/* 51: channel disconnected (closed) */

	xpBteCopyError,		/* 52: bte_copy() returned error */
	xpSalError,		/* 53: sn SAL error */
	xpRsvdPageNotSet,	/* 54: the reserved page is not set up */
	xpPayloadTooBig,	/* 55: payload too large for message slot */

	xpUnsupported,		/* 56: unsupported functionality or resource */
	xpNeedMoreInfo,		/* 57: more info is needed by SAL */

	xpGruCopyError,		/* 58: gru_copy_gru() returned error */
	xpGruSendMqError,	/* 59: gru send message queue related error */

	xpBadChannelNumber,	/* 60: invalid channel number */
	xpBadMsgType,		/* 61: invalid message type */
	xpBiosError,		/* 62: BIOS error */

	xpUnknownReason		/* 63: unknown reason - must be last in enum */
};

typedef void (*xpc_channel_func) (enum xp_retval reason, short partid,
				  int ch_number, void *data, void *key);

typedef void (*xpc_notify_func) (enum xp_retval reason, short partid,
				 int ch_number, void *key);

struct xpc_registration {
	struct mutex mutex;
	xpc_channel_func func;	/* function to call */
	void *key;		/* pointer to user's key */
	u16 nentries;		/* #of msg entries in local msg queue */
	u16 entry_size;		/* message queue's message entry size */
	u32 assigned_limit;	/* limit on #of assigned kthreads */
	u32 idle_limit;		/* limit on #of idle kthreads */
} ____cacheline_aligned;

#define XPC_CHANNEL_REGISTERED(_c)	(xpc_registrations[_c].func != NULL)

/* the following are valid xpc_send() or xpc_send_notify() flags */
#define XPC_WAIT	0	/* wait flag */
#define XPC_NOWAIT	1	/* no wait flag */

struct xpc_interface {
	void (*connect) (int);
	void (*disconnect) (int);
	enum xp_retval (*send) (short, int, u32, void *, u16);
	enum xp_retval (*send_notify) (short, int, u32, void *, u16,
					xpc_notify_func, void *);
	void (*received) (short, int, void *);
	enum xp_retval (*partid_to_nasids) (short, void *);
};

extern struct xpc_interface xpc_interface;

extern void xpc_set_interface(void (*)(int),
			      void (*)(int),
			      enum xp_retval (*)(short, int, u32, void *, u16),
			      enum xp_retval (*)(short, int, u32, void *, u16,
						 xpc_notify_func, void *),
			      void (*)(short, int, void *),
			      enum xp_retval (*)(short, void *));
extern void xpc_clear_interface(void);

extern enum xp_retval xpc_connect(int, xpc_channel_func, void *, u16,
				   u16, u32, u32);
extern void xpc_disconnect(int);

static inline enum xp_retval
xpc_send(short partid, int ch_number, u32 flags, void *payload,
	 u16 payload_size)
{
	return xpc_interface.send(partid, ch_number, flags, payload,
				  payload_size);
}

static inline enum xp_retval
xpc_send_notify(short partid, int ch_number, u32 flags, void *payload,
		u16 payload_size, xpc_notify_func func, void *key)
{
	return xpc_interface.send_notify(partid, ch_number, flags, payload,
					 payload_size, func, key);
}

static inline void
xpc_received(short partid, int ch_number, void *payload)
{
	return xpc_interface.received(partid, ch_number, payload);
}

static inline enum xp_retval
xpc_partid_to_nasids(short partid, void *nasids)
{
	return xpc_interface.partid_to_nasids(partid, nasids);
}

extern short xp_max_npartitions;
extern short xp_partition_id;
extern u8 xp_region_size;

extern unsigned long (*xp_pa) (void *);
extern unsigned long (*xp_socket_pa) (unsigned long);
extern enum xp_retval (*xp_remote_memcpy) (unsigned long, const unsigned long,
		       size_t);
extern int (*xp_cpu_to_nasid) (int);
extern enum xp_retval (*xp_expand_memprotect) (unsigned long, unsigned long);
extern enum xp_retval (*xp_restrict_memprotect) (unsigned long, unsigned long);

extern u64 xp_nofault_PIOR_target;
extern int xp_nofault_PIOR(void *);
extern int xp_error_PIOR(void);

extern struct device *xp;
extern enum xp_retval xp_init_sn2(void);
extern enum xp_retval xp_init_uv(void);
extern void xp_exit_sn2(void);
extern void xp_exit_uv(void);

#endif /* _DRIVERS_MISC_SGIXP_XP_H */
