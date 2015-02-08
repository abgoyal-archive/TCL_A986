
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_UTILS_H
#define BUS_UTILS_H

#include <dbus/dbus.h>

extern const char bus_no_memory_message[];
#define BUS_SET_OOM(error) dbus_set_error_const ((error), DBUS_ERROR_NO_MEMORY, bus_no_memory_message)

void        bus_connection_dispatch_all_messages (DBusConnection *connection);
dbus_bool_t bus_connection_dispatch_one_message  (DBusConnection *connection);

#endif /* BUS_UTILS_H */
