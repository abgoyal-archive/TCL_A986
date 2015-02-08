
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>

#include <bluetooth/bluetooth.h>

#include <gdbus.h>

#include "plugin.h"
#include "log.h"
#include "manager.h"

static GKeyFile *load_config_file(const char *file)
{
	GKeyFile *keyfile;
	GError *err = NULL;

	keyfile = g_key_file_new();

	if (!g_key_file_load_from_file(keyfile, file, 0, &err)) {
		error("Parsing %s failed: %s", file, err->message);
		g_error_free(err);
		g_key_file_free(keyfile);
		return NULL;
	}

	return keyfile;
}

static DBusConnection *connection;

static int input_init(void)
{
	GKeyFile *config;

	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	config = load_config_file(CONFIGDIR "/input.conf");

	if (input_manager_init(connection, config) < 0) {
		dbus_connection_unref(connection);
		return -EIO;
	}

	if (config)
		g_key_file_free(config);

	return 0;
}

static void input_exit(void)
{
	input_manager_exit();

	dbus_connection_unref(connection);
}

BLUETOOTH_PLUGIN_DEFINE(input, VERSION,
			BLUETOOTH_PLUGIN_PRIORITY_DEFAULT, input_init, input_exit)
