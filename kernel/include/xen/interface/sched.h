

#ifndef __XEN_PUBLIC_SCHED_H__
#define __XEN_PUBLIC_SCHED_H__

#include "event_channel.h"


#define SCHEDOP_yield       0

#define SCHEDOP_block       1

#define SCHEDOP_shutdown    2
struct sched_shutdown {
    unsigned int reason; /* SHUTDOWN_* */
};
DEFINE_GUEST_HANDLE_STRUCT(sched_shutdown);

#define SCHEDOP_poll        3
struct sched_poll {
    GUEST_HANDLE(evtchn_port_t) ports;
    unsigned int nr_ports;
    uint64_t timeout;
};
DEFINE_GUEST_HANDLE_STRUCT(sched_poll);

#define SHUTDOWN_poweroff   0  /* Domain exited normally. Clean up and kill. */
#define SHUTDOWN_reboot     1  /* Clean up, kill, and then restart.          */
#define SHUTDOWN_suspend    2  /* Clean up, save suspend info, kill.         */
#define SHUTDOWN_crash      3  /* Tell controller we've crashed.             */

#endif /* __XEN_PUBLIC_SCHED_H__ */
