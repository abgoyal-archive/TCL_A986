LOCAL_C_INCLUDES := \
	external/libffi/include \
	external/libffi/$(ffi_os)-$(ffi_arch)

ifeq ($(ffi_os)-$(ffi_arch),linux-arm)
  LOCAL_SRC_FILES := src/arm/sysv.S src/arm/ffi.c
endif

ifeq ($(ffi_os)-$(ffi_arch),linux-x86)
  LOCAL_SRC_FILES := src/x86/ffi.c src/x86/sysv.S
endif

ifeq ($(ffi_os)-$(ffi_arch),linux-sh)
  LOCAL_SRC_FILES := src/sh/ffi.c src/sh/sysv.S
endif

ifeq ($(ffi_os)-$(ffi_arch),darwin-x86)
  LOCAL_SRC_FILES := src/x86/ffi.c src/x86/darwin.S
endif

ifeq ($(LOCAL_SRC_FILES),)
  $(info The os/architecture $(ffi_os)-$(ffi_arch) is not supported by libffi.)
  LOCAL_SRC_FILES := your-architecture-not-supported-by-ffi-makefile.c
endif

LOCAL_SRC_FILES += \
	src/debug.c \
	src/java_raw_api.c \
	src/prep_cif.c \
        src/raw_api.c \
	src/types.c
