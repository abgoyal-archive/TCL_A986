
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>

#include <gdbus.h>

#include "plugin.h"
#include "manager.h"

static DBusConnection *connection;

static int serial_init(void)
{
	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (serial_manager_init(connection) < 0) {
		dbus_connection_unref(connection);
		return -EIO;
	}

	return 0;
}

static void serial_exit(void)
{
	serial_manager_exit();

	dbus_connection_unref(connection);
}

BLUETOOTH_PLUGIN_DEFINE(serial, VERSION,
			BLUETOOTH_PLUGIN_PRIORITY_DEFAULT, serial_init, serial_exit)
