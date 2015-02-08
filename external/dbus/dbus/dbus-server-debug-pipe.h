
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_SERVER_DEBUG_PIPE_H
#define DBUS_SERVER_DEBUG_PIPE_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-server-protected.h>
#include <dbus/dbus-transport-protected.h>

DBUS_BEGIN_DECLS

DBusServer*             _dbus_server_debug_pipe_new     (const char        *server_name,
                                                         DBusError         *error);
DBusTransport*          _dbus_transport_debug_pipe_new  (const char        *server_name,
                                                         DBusError         *error);
DBusServerListenResult  _dbus_server_listen_debug_pipe  (DBusAddressEntry  *entry,
                                                         DBusServer       **server_p,
                                                         DBusError         *error);
DBusTransportOpenResult _dbus_transport_open_debug_pipe (DBusAddressEntry  *entry,
                                                         DBusTransport    **transport_p,
                                                         DBusError         *error);


DBUS_END_DECLS

#endif /* DBUS_SERVER_DEBUG_PIPE_H */
