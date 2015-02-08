
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_RESOURCES_H
#define DBUS_RESOURCES_H

#include <dbus/dbus-macros.h>
#include <dbus/dbus-errors.h>
#include <dbus/dbus-connection.h>

DBUS_BEGIN_DECLS

typedef struct DBusCounter DBusCounter;

typedef void (* DBusCounterNotifyFunction) (DBusCounter *counter,
                                            void        *user_data);

DBusCounter* _dbus_counter_new       (void);
DBusCounter* _dbus_counter_ref       (DBusCounter *counter);
void         _dbus_counter_unref     (DBusCounter *counter);
void         _dbus_counter_adjust    (DBusCounter *counter,
                                      long         delta);
long         _dbus_counter_get_value (DBusCounter *counter);

void _dbus_counter_set_notify (DBusCounter               *counter,
                               long                       guard_value,
                               DBusCounterNotifyFunction  function,
                               void                      *user_data);


DBUS_END_DECLS

#endif /* DBUS_RESOURCES_H */
