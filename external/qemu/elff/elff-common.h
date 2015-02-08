


#ifndef ELFF_ELFF_COMMON_H_
#define ELFF_ELFF_COMMON_H_

#include "stddef.h"
#include "sys/types.h"
#include "assert.h"
#include "memory.h"
#include "errno.h"
#ifdef  WIN32
#include "windows.h"
#else   // WIN32
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif  // WIN32

static inline void _set_errno(uint32_t err) {
    errno = err;
}

inline void* operator new(size_t size) {
    return qemu_malloc(size);
}

inline void operator delete(void* p) {
    if (p != NULL) {
        qemu_free(p);
    }
}

inline void operator delete[](void* p) {
    if (p != NULL) {
        qemu_free(p);
    }
}

#endif  // ELFF_ELFF_COMMON_H_
