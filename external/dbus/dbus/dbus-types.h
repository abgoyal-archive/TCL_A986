
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_TYPES_H
#define DBUS_TYPES_H

#include <stddef.h>
#include <dbus/dbus-arch-deps.h>

typedef dbus_uint32_t  dbus_unichar_t;
/* boolean size must be fixed at 4 bytes due to wire protocol! */
typedef dbus_uint32_t  dbus_bool_t;

/* Normally docs are in .c files, but there isn't a .c file for this. */












/** @} */

#endif /* DBUS_TYPES_H */
