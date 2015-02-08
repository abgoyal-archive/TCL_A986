
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_AUTH_SCRIPT_H
#define DBUS_AUTH_SCRIPT_H

#include <config.h>

#include <dbus/dbus-memory.h>
#include <dbus/dbus-types.h>
#include <dbus/dbus-string.h>

DBUS_BEGIN_DECLS

dbus_bool_t _dbus_auth_script_run (const DBusString *filename);

DBUS_END_DECLS

#endif /* DBUS_AUTH_SCRIPT_H */
