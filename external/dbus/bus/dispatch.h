
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_DISPATCH_H
#define BUS_DISPATCH_H

#include <dbus/dbus.h>
#include "connection.h"

dbus_bool_t bus_dispatch_add_connection    (DBusConnection *connection);
void        bus_dispatch_remove_connection (DBusConnection *connection);
dbus_bool_t bus_dispatch_matches           (BusTransaction *transaction,
                                            DBusConnection *sender,
                                            DBusConnection *recipient,
                                            DBusMessage    *message,
                                            DBusError      *error);

#endif /* BUS_DISPATCH_H */
