
#include "qemu-common.h"
#include <stdlib.h>

static void *oom_check(void *ptr)
{
    if (ptr == NULL) {
        abort();
    }
    return ptr;
}

void *get_mmap_addr(unsigned long size)
{
    return NULL;
}

void qemu_free(void *ptr)
{
    free(ptr);
}

static int allow_zero_malloc(void)
{
#if defined(CONFIG_ZERO_MALLOC)
    return 1;
#else
    return 0;
#endif
}

void *qemu_malloc(size_t size)
{
    if (!size && !allow_zero_malloc()) {
        abort();
    }
    return oom_check(malloc(size ? size : 1));
}

void *qemu_realloc(void *ptr, size_t size)
{
    if (size) {
        return oom_check(realloc(ptr, size));
    } else if (allow_zero_malloc()) {
        return oom_check(realloc(ptr, size ? size : 1));
    }
    abort();
}

void *qemu_mallocz(size_t size)
{
    void *ptr;
    ptr = qemu_malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

char *qemu_strdup(const char *str)
{
    char *ptr;
    size_t len = strlen(str);
    ptr = qemu_malloc(len + 1);
    memcpy(ptr, str, len + 1);
    return ptr;
}

char *qemu_strndup(const char *str, size_t size)
{
    const char *end = memchr(str, 0, size);
    char *new;

    if (end) {
        size = end - str;
    }

    new = qemu_malloc(size + 1);
    new[size] = 0;

    return memcpy(new, str, size);
}
