
#ifndef OP_LOCKFILE_H
#define OP_LOCKFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>

int op_write_lock_file(char const * file);

#ifdef __cplusplus
}
#endif

#endif /* OP_LOCKFILE_H */
