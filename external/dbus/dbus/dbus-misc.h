
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_MISC_H
#define DBUS_MISC_H

#include <dbus/dbus-types.h>
#include <dbus/dbus-errors.h>

DBUS_BEGIN_DECLS


char*       dbus_get_local_machine_id              (void);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_MISC_H */

