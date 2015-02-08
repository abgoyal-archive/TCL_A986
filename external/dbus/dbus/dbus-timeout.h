
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_TIMEOUT_H
#define DBUS_TIMEOUT_H

#include <dbus/dbus-connection.h>
#include <dbus/dbus-internals.h>

DBUS_BEGIN_DECLS


/* Public methods on DBusTimeout are in dbus-connection.h */

typedef struct DBusTimeoutList DBusTimeoutList;

/** function to run when the timeout is handled */
typedef dbus_bool_t (* DBusTimeoutHandler) (void *data);

DBusTimeout* _dbus_timeout_new          (int                 interval,
                                         DBusTimeoutHandler  handler,
                                         void               *data,
                                         DBusFreeFunction    free_data_function);
DBusTimeout* _dbus_timeout_ref          (DBusTimeout        *timeout);
void         _dbus_timeout_unref        (DBusTimeout        *timeout);
void         _dbus_timeout_set_interval (DBusTimeout        *timeout,
                                         int                 interval);
void         _dbus_timeout_set_enabled  (DBusTimeout        *timeout,
                                         dbus_bool_t         enabled);

DBusTimeoutList *_dbus_timeout_list_new            (void);
void             _dbus_timeout_list_free           (DBusTimeoutList           *timeout_list);
dbus_bool_t      _dbus_timeout_list_set_functions  (DBusTimeoutList           *timeout_list,
						    DBusAddTimeoutFunction     add_function,
						    DBusRemoveTimeoutFunction  remove_function,
                                                    DBusTimeoutToggledFunction toggled_function,
						    void                      *data,
						    DBusFreeFunction           free_data_function);
dbus_bool_t      _dbus_timeout_list_add_timeout    (DBusTimeoutList           *timeout_list,
						    DBusTimeout               *timeout);
void             _dbus_timeout_list_remove_timeout (DBusTimeoutList           *timeout_list,
						    DBusTimeout               *timeout);
void             _dbus_timeout_list_toggle_timeout (DBusTimeoutList           *timeout_list,
                                                    DBusTimeout               *timeout,
                                                    dbus_bool_t                enabled);


/** @} */

DBUS_END_DECLS

#endif /* DBUS_TIMEOUT_H */
