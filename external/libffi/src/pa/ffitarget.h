
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

/* ---- System specific configurations ----------------------------------- */

#ifndef LIBFFI_ASM
typedef unsigned long          ffi_arg;
typedef signed long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,

#ifdef PA_LINUX
  FFI_PA32,
  FFI_DEFAULT_ABI = FFI_PA32,
#endif

#ifdef PA_HPUX
  FFI_PA32,
  FFI_DEFAULT_ABI = FFI_PA32,
#endif

#ifdef PA64_HPUX
#error "PA64_HPUX FFI is not yet implemented"
  FFI_PA64,
  FFI_DEFAULT_ABI = FFI_PA64,
#endif

  FFI_LAST_ABI = FFI_DEFAULT_ABI + 1
} ffi_abi;
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_NATIVE_RAW_API 0

#ifdef PA_LINUX
#define FFI_TRAMPOLINE_SIZE 32
#else
#define FFI_TRAMPOLINE_SIZE 40
#endif

#define FFI_TYPE_SMALL_STRUCT2 -1
#define FFI_TYPE_SMALL_STRUCT3 -2
#define FFI_TYPE_SMALL_STRUCT4 -3
#define FFI_TYPE_SMALL_STRUCT5 -4
#define FFI_TYPE_SMALL_STRUCT6 -5
#define FFI_TYPE_SMALL_STRUCT7 -6
#define FFI_TYPE_SMALL_STRUCT8 -7
#endif
