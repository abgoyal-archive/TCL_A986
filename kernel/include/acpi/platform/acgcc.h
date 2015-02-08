


#ifndef __ACGCC_H__
#define __ACGCC_H__

/* Function name is used for debug output. Non-ANSI, compiler-dependent */

#define ACPI_GET_FUNCTION_NAME          __func__

#define ACPI_PRINTF_LIKE(c) __attribute__ ((__format__ (__printf__, c, c+1)))

#define ACPI_UNUSED_VAR __attribute__ ((unused))

#ifdef _ANSI
#define inline
#endif

#endif				/* __ACGCC_H__ */
