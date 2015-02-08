
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gdbus.h>

#include "error.h"

/* Helper function - internal use only */
DBusHandlerResult error_common_reply(DBusConnection *conn, DBusMessage *msg,
					const char *name, const char *descr)
{
	DBusMessage *derr;

	if (!conn || !msg)
		return DBUS_HANDLER_RESULT_HANDLED;

	derr = dbus_message_new_error(msg, name, descr);
	if (!derr)
		return DBUS_HANDLER_RESULT_NEED_MEMORY;

	g_dbus_send_message(conn, derr);

	return DBUS_HANDLER_RESULT_HANDLED;
}
