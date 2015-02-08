
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_ARCH_DEPS_H
#define DBUS_ARCH_DEPS_H

#include <dbus/dbus-macros.h>

DBUS_BEGIN_DECLS

#if 1
#define DBUS_HAVE_INT64 1
typedef long long dbus_int64_t;
typedef unsigned long long dbus_uint64_t;

#define DBUS_INT64_CONSTANT(val)  (val##LL)
#define DBUS_UINT64_CONSTANT(val) (val##ULL)

#else
#undef DBUS_HAVE_INT64
#undef DBUS_INT64_CONSTANT
#undef DBUS_UINT64_CONSTANT
#endif

typedef int dbus_int32_t;
typedef unsigned int dbus_uint32_t;

typedef short dbus_int16_t;
typedef unsigned short dbus_uint16_t;

DBUS_END_DECLS

#endif /* DBUS_ARCH_DEPS_H */
