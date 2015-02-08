
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_THREADS_INTERNAL_H
#define DBUS_THREADS_INTERNAL_H

#include <dbus/dbus-macros.h>
#include <dbus/dbus-types.h>
#include <dbus/dbus-threads.h>

DBUS_BEGIN_DECLS

DBusMutex*   _dbus_mutex_new                 (void);
void         _dbus_mutex_free                (DBusMutex         *mutex);
void         _dbus_mutex_lock                (DBusMutex         *mutex);
void         _dbus_mutex_unlock              (DBusMutex         *mutex);
void         _dbus_mutex_new_at_location     (DBusMutex        **location_p);
void         _dbus_mutex_free_at_location    (DBusMutex        **location_p);

DBusCondVar* _dbus_condvar_new               (void);
void         _dbus_condvar_free              (DBusCondVar       *cond);
void         _dbus_condvar_wait              (DBusCondVar       *cond,
                                              DBusMutex         *mutex);
dbus_bool_t  _dbus_condvar_wait_timeout      (DBusCondVar       *cond,
                                              DBusMutex         *mutex,
                                              int                timeout_milliseconds);
void         _dbus_condvar_wake_one          (DBusCondVar       *cond);
void         _dbus_condvar_wake_all          (DBusCondVar       *cond);
void         _dbus_condvar_new_at_location   (DBusCondVar      **location_p);
void         _dbus_condvar_free_at_location  (DBusCondVar      **location_p);

DBUS_END_DECLS

#endif /* DBUS_THREADS_INTERNAL_H */
