

#ifndef QEMU_AIO_H
#define QEMU_AIO_H

#include "qemu-common.h"
#include "qemu-char.h"

/* Returns 1 if there are still outstanding AIO requests; 0 otherwise */
typedef int (AioFlushHandler)(void *opaque);

typedef int (AioProcessQueue)(void *opaque);

void qemu_aio_flush(void);

void qemu_aio_wait(void);

int qemu_aio_set_fd_handler(int fd,
                            IOHandler *io_read,
                            IOHandler *io_write,
                            AioFlushHandler *io_flush,
                            void *opaque);

#endif
