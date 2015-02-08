
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifdef DBUS_INSIDE_DBUS_H
#error "You can't include dbus-uuidgen.h in the public header dbus.h"
#endif

#ifndef DBUS_UUIDGEN_H
#define DBUS_UUIDGEN_H

#include <dbus/dbus-types.h>
#include <dbus/dbus-errors.h>

DBUS_BEGIN_DECLS

dbus_bool_t dbus_internal_do_not_use_get_uuid    (const char *filename,
                                                  char      **uuid_p,
                                                  dbus_bool_t create_if_not_found,
                                                  DBusError  *error);
dbus_bool_t dbus_internal_do_not_use_ensure_uuid (const char *filename,
                                                  char      **uuid_p,
                                                  DBusError  *error);
dbus_bool_t dbus_internal_do_not_use_create_uuid (char      **uuid_p);


DBUS_END_DECLS

#endif /* DBUS_UUIDGEN_H */
