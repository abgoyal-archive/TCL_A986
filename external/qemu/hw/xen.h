
#ifndef QEMU_HW_XEN_H
#define QEMU_HW_XEN_H 1
#include <inttypes.h>

/* xen-machine.c */
enum xen_mode {
    XEN_EMULATE = 0,  // xen emulation, using xenner (default)
    XEN_CREATE,       // create xen domain
    XEN_ATTACH        // attach to xen domain created by xend
};

extern uint32_t xen_domid;
extern enum xen_mode xen_mode;

#endif /* QEMU_HW_XEN_H */
