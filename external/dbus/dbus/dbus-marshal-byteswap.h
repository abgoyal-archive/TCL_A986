
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_MARSHAL_BYTESWAP_H
#define DBUS_MARSHAL_BYTESWAP_H

#include <config.h>
#include <dbus/dbus-protocol.h>
#include <dbus/dbus-marshal-recursive.h>

#ifndef PACKAGE
#error "config.h not included here"
#endif

void _dbus_marshal_byteswap (const DBusString *signature,
                             int               signature_start,
                             int               old_byte_order,
                             int               new_byte_order,
                             DBusString       *value_str,
                             int               value_pos);

#endif /* DBUS_MARSHAL_BYTESWAP_H */
