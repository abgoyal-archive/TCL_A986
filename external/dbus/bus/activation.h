
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_ACTIVATION_H
#define BUS_ACTIVATION_H

#include <dbus/dbus.h>
#include <dbus/dbus-list.h>
#include "bus.h"

BusActivation* bus_activation_new              (BusContext        *context,
						const DBusString  *address,
						DBusList         **directories,
						DBusError         *error);
BusActivation* bus_activation_ref              (BusActivation     *activation);
void           bus_activation_unref            (BusActivation     *activation);
dbus_bool_t    bus_activation_activate_service (BusActivation     *activation,
						DBusConnection    *connection,
						BusTransaction    *transaction,
						dbus_bool_t        auto_activation,
						DBusMessage       *activation_message,
						const char        *service_name,
						DBusError         *error);
dbus_bool_t    bus_activation_service_created  (BusActivation     *activation,
						const char        *service_name,
						BusTransaction    *transaction,
						DBusError         *error);
dbus_bool_t    bus_activation_list_services    (BusActivation     *registry,
						char            ***listp,
						int               *array_len);

dbus_bool_t    bus_activation_send_pending_auto_activation_messages (BusActivation     *activation,
								     BusService        *service,
								     BusTransaction    *transaction,
								     DBusError         *error);



#endif /* BUS_ACTIVATION_H */
