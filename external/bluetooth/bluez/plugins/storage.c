
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <bluetooth/bluetooth.h>

#include "plugin.h"
#include "log.h"

static int storage_init(void)
{
	return 0;
}

static void storage_exit(void)
{
}

BLUETOOTH_PLUGIN_DEFINE(storage, VERSION,
		BLUETOOTH_PLUGIN_PRIORITY_DEFAULT, storage_init, storage_exit)
