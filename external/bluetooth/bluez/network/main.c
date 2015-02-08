
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>

#include <gdbus.h>

#include "plugin.h"
#include "manager.h"

static DBusConnection *connection;

static int network_init(void)
{
	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (network_manager_init(connection) < 0) {
		dbus_connection_unref(connection);
		return -EIO;
	}

	return 0;
}

static void network_exit(void)
{
	network_manager_exit();

	dbus_connection_unref(connection);
}

BLUETOOTH_PLUGIN_DEFINE(network, VERSION,
			BLUETOOTH_PLUGIN_PRIORITY_DEFAULT, network_init, network_exit)
