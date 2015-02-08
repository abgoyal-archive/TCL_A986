
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_SERVER_SOCKET_H
#define DBUS_SERVER_SOCKET_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-server-protected.h>

DBUS_BEGIN_DECLS

DBusServer* _dbus_server_new_for_socket           (int               fd,
                                                   const DBusString *address);
DBusServer* _dbus_server_new_for_tcp_socket       (const char       *host,
                                                   dbus_uint32_t     port,
                                                   DBusError        *error);
DBusServerListenResult _dbus_server_listen_socket (DBusAddressEntry  *entry,
                                                   DBusServer       **server_p,
                                                   DBusError         *error);


void _dbus_server_socket_own_filename (DBusServer *server,
                                       char       *filename);

DBUS_END_DECLS

#endif /* DBUS_SERVER_SOCKET_H */
