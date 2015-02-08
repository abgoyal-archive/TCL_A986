
/* -*- mode: C; c-file-style: "gnu" -*- */

#include <config.h>
#include "utils.h"
#include <dbus/dbus-sysdeps.h>
#include <dbus/dbus-mainloop.h>

const char bus_no_memory_message[] = "Memory allocation failure in message bus";

void
bus_connection_dispatch_all_messages (DBusConnection *connection)
{
  while (bus_connection_dispatch_one_message (connection))
    ;
}

dbus_bool_t
bus_connection_dispatch_one_message  (DBusConnection *connection)
{
  DBusDispatchStatus status;

  while ((status = dbus_connection_dispatch (connection)) == DBUS_DISPATCH_NEED_MEMORY)
    _dbus_wait_for_memory ();
  
  return status == DBUS_DISPATCH_DATA_REMAINS;
}
