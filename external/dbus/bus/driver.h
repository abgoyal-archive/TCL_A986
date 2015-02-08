
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_DRIVER_H
#define BUS_DRIVER_H

#include <dbus/dbus.h>
#include "connection.h"

void        bus_driver_remove_connection     (DBusConnection *connection);
dbus_bool_t bus_driver_handle_message        (DBusConnection *connection,
                                              BusTransaction *transaction,
                                              DBusMessage    *message,
                                              DBusError      *error);
dbus_bool_t bus_driver_send_service_lost     (DBusConnection *connection,
                                              const char     *service_name,
                                              BusTransaction *transaction,
                                              DBusError      *error);
dbus_bool_t bus_driver_send_service_acquired (DBusConnection *connection,
                                              const char     *service_name,
                                              BusTransaction *transaction,
                                              DBusError      *error);
dbus_bool_t bus_driver_send_service_owner_changed  (const char     *service_name,
						    const char     *old_owner,
						    const char     *new_owner,
						    BusTransaction *transaction,
						    DBusError      *error);
dbus_bool_t bus_driver_generate_introspect_string  (DBusString *xml);



#endif /* BUS_DRIVER_H */
