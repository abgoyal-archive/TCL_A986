
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_PENDING_CALL_INTERNAL_H
#define DBUS_PENDING_CALL_INTERNAL_H


#include <dbus/dbus-internals.h>
#include <dbus/dbus-message.h>
#include <dbus/dbus-connection.h>
#include <dbus/dbus-list.h>

DBUS_BEGIN_DECLS

dbus_bool_t      _dbus_pending_call_is_timeout_added_unlocked    (DBusPendingCall    *pending);
void             _dbus_pending_call_set_timeout_added_unlocked   (DBusPendingCall    *pending,
                                                                  dbus_bool_t         is_added);
DBusTimeout    * _dbus_pending_call_get_timeout_unlocked         (DBusPendingCall    *pending);
dbus_uint32_t    _dbus_pending_call_get_reply_serial_unlocked    (DBusPendingCall    *pending);
void             _dbus_pending_call_set_reply_serial_unlocked    (DBusPendingCall    *pending,
                                                                  dbus_uint32_t       serial);
DBusConnection * _dbus_pending_call_get_connection_and_lock      (DBusPendingCall    *pending);
DBusConnection * _dbus_pending_call_get_connection_unlocked      (DBusPendingCall    *pending);
dbus_bool_t      _dbus_pending_call_get_completed_unlocked       (DBusPendingCall    *pending);
void             _dbus_pending_call_complete                     (DBusPendingCall    *pending);
void             _dbus_pending_call_set_reply_unlocked           (DBusPendingCall    *pending,
                                                                  DBusMessage        *message);
void             _dbus_pending_call_queue_timeout_error_unlocked (DBusPendingCall    *pending,
                                                                  DBusConnection     *connection);
void             _dbus_pending_call_set_reply_serial_unlocked    (DBusPendingCall    *pending,
                                                                  dbus_uint32_t       serial);
dbus_bool_t      _dbus_pending_call_set_timeout_error_unlocked   (DBusPendingCall    *pending,
                                                                  DBusMessage        *message,
                                                                  dbus_uint32_t       serial);
DBusPendingCall* _dbus_pending_call_new_unlocked                 (DBusConnection     *connection,
                                                                  int                 timeout_milliseconds,
                                                                  DBusTimeoutHandler  timeout_handler);
DBusPendingCall* _dbus_pending_call_ref_unlocked                 (DBusPendingCall    *pending);
void             _dbus_pending_call_unref_and_unlock             (DBusPendingCall    *pending);
dbus_bool_t      _dbus_pending_call_set_data_unlocked            (DBusPendingCall    *pending,
                                                                  dbus_int32_t        slot,
                                                                  void               *data,
                                                                  DBusFreeFunction    free_data_func);


DBUS_END_DECLS

#endif /* DBUS_PENDING_CALL_INTERNAL_H */
