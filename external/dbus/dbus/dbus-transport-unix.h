
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_TRANSPORT_UNIX_H
#define DBUS_TRANSPORT_UNIX_H

#include <dbus/dbus-transport.h>

DBUS_BEGIN_DECLS

DBusTransport* _dbus_transport_new_for_domain_socket (const char       *path,
                                                      dbus_bool_t       abstract,
                                                      DBusError        *error);


DBUS_END_DECLS

#endif /* DBUS_TRANSPORT_UNIX_H */
