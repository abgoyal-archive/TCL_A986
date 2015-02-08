

#ifndef QEMU_BUFFERED_FILE_H
#define QEMU_BUFFERED_FILE_H

#include "hw/hw.h"

typedef ssize_t (BufferedPutFunc)(void *opaque, const void *data, size_t size);
typedef void (BufferedPutReadyFunc)(void *opaque);
typedef void (BufferedWaitForUnfreezeFunc)(void *opaque);
typedef int (BufferedCloseFunc)(void *opaque);

QEMUFile *qemu_fopen_ops_buffered(void *opaque, size_t xfer_limit,
                                  BufferedPutFunc *put_buffer,
                                  BufferedPutReadyFunc *put_ready,
                                  BufferedWaitForUnfreezeFunc *wait_for_unfreeze,
                                  BufferedCloseFunc *close);

#endif
