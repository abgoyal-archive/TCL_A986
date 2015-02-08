
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_TRANSPORT_H
#define DBUS_TRANSPORT_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-connection.h>
#include <dbus/dbus-protocol.h>
#include <dbus/dbus-address.h>

DBUS_BEGIN_DECLS

typedef struct DBusTransport DBusTransport;

DBusTransport*     _dbus_transport_open                   (DBusAddressEntry           *entry,
                                                           DBusError                  *error);
DBusTransport*     _dbus_transport_ref                    (DBusTransport              *transport);
void               _dbus_transport_unref                  (DBusTransport              *transport);
void               _dbus_transport_disconnect             (DBusTransport              *transport);
dbus_bool_t        _dbus_transport_get_is_connected       (DBusTransport              *transport);
dbus_bool_t        _dbus_transport_get_is_authenticated   (DBusTransport              *transport);
const char*        _dbus_transport_get_address            (DBusTransport              *transport);
dbus_bool_t        _dbus_transport_handle_watch           (DBusTransport              *transport,
                                                           DBusWatch                  *watch,
                                                           unsigned int                condition);
dbus_bool_t        _dbus_transport_set_connection         (DBusTransport              *transport,
                                                           DBusConnection             *connection);
void               _dbus_transport_do_iteration           (DBusTransport              *transport,
                                                           unsigned int                flags,
                                                           int                         timeout_milliseconds);
DBusDispatchStatus _dbus_transport_get_dispatch_status    (DBusTransport              *transport);
dbus_bool_t        _dbus_transport_queue_messages         (DBusTransport              *transport);
void               _dbus_transport_set_max_message_size   (DBusTransport              *transport,
                                                           long                        size);
long               _dbus_transport_get_max_message_size   (DBusTransport              *transport);
void               _dbus_transport_set_max_received_size  (DBusTransport              *transport,
                                                           long                        size);
long               _dbus_transport_get_max_received_size  (DBusTransport              *transport);
dbus_bool_t        _dbus_transport_get_socket_fd          (DBusTransport              *transport,
                                                           int                        *fd_p);
dbus_bool_t        _dbus_transport_get_unix_user          (DBusTransport              *transport,
                                                           unsigned long              *uid);
dbus_bool_t        _dbus_transport_get_unix_process_id     (DBusTransport              *transport,
                                                           unsigned long              *pid);
void               _dbus_transport_set_unix_user_function (DBusTransport              *transport,
                                                           DBusAllowUnixUserFunction   function,
                                                           void                       *data,
                                                           DBusFreeFunction            free_data_function,
                                                           void                      **old_data,
                                                           DBusFreeFunction           *old_free_data_function);
dbus_bool_t        _dbus_transport_set_auth_mechanisms    (DBusTransport              *transport,
                                                           const char                **mechanisms);




DBUS_END_DECLS

#endif /* DBUS_TRANSPORT_H */
