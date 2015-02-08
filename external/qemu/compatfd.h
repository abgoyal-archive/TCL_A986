

#ifndef QEMU_COMPATFD_H
#define QEMU_COMPATFD_H

#include <signal.h>

struct qemu_signalfd_siginfo {
    uint32_t ssi_signo;
    uint8_t pad[124];
};

int qemu_signalfd(const sigset_t *mask);

int qemu_eventfd(int *fds);

#endif
