
#include <dbus/dbus.h>

#define ERROR_INTERFACE "org.bluez.Error"

DBusHandlerResult error_common_reply(DBusConnection *conn, DBusMessage *msg,
					const char *name, const char *descr);
