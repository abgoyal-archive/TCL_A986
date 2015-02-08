

#ifndef __XEN_PUBLIC_CALLBACK_H__
#define __XEN_PUBLIC_CALLBACK_H__

#include "xen.h"


/* ia64, x86: Callback for event delivery. */
#define CALLBACKTYPE_event                 0

/* x86: Failsafe callback when guest state cannot be restored by Xen. */
#define CALLBACKTYPE_failsafe              1

/* x86/64 hypervisor: Syscall by 64-bit guest app ('64-on-64-on-64'). */
#define CALLBACKTYPE_syscall               2

#define CALLBACKTYPE_sysenter_deprecated   3

/* x86: Callback for NMI delivery. */
#define CALLBACKTYPE_nmi                   4

#define CALLBACKTYPE_sysenter              5

#define CALLBACKTYPE_syscall32             7

#define _CALLBACKF_mask_events             0
#define CALLBACKF_mask_events              (1U << _CALLBACKF_mask_events)

#define CALLBACKOP_register                0
struct callback_register {
	uint16_t type;
	uint16_t flags;
	xen_callback_t address;
};

#define CALLBACKOP_unregister              1
struct callback_unregister {
    uint16_t type;
    uint16_t _unused;
};

#endif /* __XEN_PUBLIC_CALLBACK_H__ */
