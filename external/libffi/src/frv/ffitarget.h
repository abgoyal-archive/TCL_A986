
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

/* ---- System specific configurations ----------------------------------- */

#ifndef LIBFFI_ASM
typedef unsigned long          ffi_arg;
typedef signed long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,

#ifdef FRV
  FFI_EABI,
  FFI_DEFAULT_ABI = FFI_EABI,
#endif

  FFI_LAST_ABI = FFI_DEFAULT_ABI + 1
} ffi_abi;
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_NATIVE_RAW_API 0

#ifdef __FRV_FDPIC__
/* Trampolines are 8 4-byte instructions long.  */
#define FFI_TRAMPOLINE_SIZE (8*4)
#else
/* Trampolines are 5 4-byte instructions long.  */
#define FFI_TRAMPOLINE_SIZE (5*4)
#endif

#endif
