


#ifndef QEMU_MEMCHECK_MEMCHECK_LOGGING_H
#define QEMU_MEMCHECK_MEMCHECK_LOGGING_H

#ifndef CONFIG_MEMCHECK
#error CONFIG_MEMCHECK is not defined.
#endif  // CONFIG_MEMCHECK

#include "qemu-common.h"
#include "android/utils/debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Prints debug message under the 'memcheck' tag. */
#define MD(...)  VERBOSE_PRINT(memcheck, __VA_ARGS__)

/* Prints an error message under the 'memcheck' tag. */
#define ME(...)  \
    do { if (VERBOSE_CHECK(memcheck)) derror(__VA_ARGS__); } while (0)

// =============================================================================
// Tracing flags (see trace_flags declared in memcheck.c), and macros
// =============================================================================

/* Enables fork() tracing. */
#define TRACE_PROC_FORK_ENABLED                 0x00000001
/* Enables clone() tracing. */
#define TRACE_PROC_CLONE_ENABLED                0x00000002
/* Enables new PID allocation tracing. */
#define TRACE_PROC_NEW_PID_ENABLED              0x00000004
/* Enables guest process starting tracing. */
#define TRACE_PROC_START_ENABLED                0x00000008
/* Enables guest process exiting tracing. */
#define TRACE_PROC_EXIT_ENABLED                 0x00000010
/* Enables libc.so initialization tracing. */
#define TRACE_PROC_LIBC_INIT_ENABLED            0x00000020
/* Enables leaking tracing. */
#define TRACE_CHECK_LEAK_ENABLED                0x00000040
/* Enables invalid pointer access tracing. */
#define TRACE_CHECK_INVALID_PTR_ENABLED         0x00000080
/* Enables reading violations tracing. */
#define TRACE_CHECK_READ_VIOLATION_ENABLED      0x00000100
/* Enables writing violations tracing. */
#define TRACE_CHECK_WRITE_VIOLATION_ENABLED     0x00000200
/* Enables module mapping tracing. */
#define TRACE_PROC_MMAP_ENABLED                 0x00000400
/* All tracing flags combined. */
#define TRACE_ALL_ENABLED   (TRACE_PROC_FORK_ENABLED                | \
                             TRACE_PROC_CLONE_ENABLED               | \
                             TRACE_PROC_NEW_PID_ENABLED             | \
                             TRACE_PROC_START_ENABLED               | \
                             TRACE_PROC_LIBC_INIT_ENABLED           | \
                             TRACE_PROC_EXIT_ENABLED                | \
                             TRACE_CHECK_INVALID_PTR_ENABLED        | \
                             TRACE_CHECK_READ_VIOLATION_ENABLED     | \
                             TRACE_CHECK_WRITE_VIOLATION_ENABLED    | \
                             TRACE_PROC_MMAP_ENABLED                | \
                             TRACE_CHECK_LEAK_ENABLED)

/* Prints a trace to the stdout. */
#define T(level, ...)                                   \
    do {                                                \
        if (trace_flags & TRACE_##level##_ENABLED) {    \
            printf(__VA_ARGS__);                        \
        }                                               \
    } while (0)

/* Set of tracing flags (declared in memchek.c). */
extern uint32_t trace_flags;

#ifdef __cplusplus
};  /* end of extern "C" */
#endif

#endif  // QEMU_MEMCHECK_MEMCHECK_LOGGING_H
