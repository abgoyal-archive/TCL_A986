
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_PENDING_CALL_H
#define DBUS_PENDING_CALL_H

#include <dbus/dbus-macros.h>
#include <dbus/dbus-types.h>
#include <dbus/dbus-connection.h>

DBUS_BEGIN_DECLS


DBusPendingCall* dbus_pending_call_ref       (DBusPendingCall               *pending);
void         dbus_pending_call_unref         (DBusPendingCall               *pending);
dbus_bool_t  dbus_pending_call_set_notify    (DBusPendingCall               *pending,
                                              DBusPendingCallNotifyFunction  function,
                                              void                          *user_data,
                                              DBusFreeFunction               free_user_data);
void         dbus_pending_call_cancel        (DBusPendingCall               *pending);
dbus_bool_t  dbus_pending_call_get_completed (DBusPendingCall               *pending);
DBusMessage* dbus_pending_call_steal_reply   (DBusPendingCall               *pending);
void         dbus_pending_call_block         (DBusPendingCall               *pending);

dbus_bool_t dbus_pending_call_allocate_data_slot (dbus_int32_t     *slot_p);
void        dbus_pending_call_free_data_slot     (dbus_int32_t     *slot_p);
dbus_bool_t dbus_pending_call_set_data           (DBusPendingCall  *pending,
                                                  dbus_int32_t      slot,
                                                  void             *data,
                                                  DBusFreeFunction  free_data_func);
void*       dbus_pending_call_get_data           (DBusPendingCall  *pending,
                                                  dbus_int32_t      slot);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_PENDING_CALL_H */
