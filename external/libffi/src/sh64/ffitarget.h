
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

/* ---- Generic type definitions ----------------------------------------- */

#ifndef LIBFFI_ASM
typedef unsigned long          ffi_arg;
typedef signed long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,
  FFI_SYSV,
  FFI_DEFAULT_ABI = FFI_SYSV,
  FFI_LAST_ABI = FFI_DEFAULT_ABI + 1
} ffi_abi;

#define FFI_EXTRA_CIF_FIELDS long long flags2
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_TRAMPOLINE_SIZE 32
#define FFI_NATIVE_RAW_API 0

#endif

