
#ifndef OP_DEVICEIO_H
#define OP_DEVICEIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "op_types.h"

#include <unistd.h>

fd_t op_open_device(char const * name);

ssize_t op_read_device(fd_t devfd, void * buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* OP_DEVICEIO_H */
