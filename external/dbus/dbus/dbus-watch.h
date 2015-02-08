
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_WATCH_H
#define DBUS_WATCH_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-connection.h>

DBUS_BEGIN_DECLS


/* Public methods on DBusWatch are in dbus-connection.h */

typedef struct DBusWatchList DBusWatchList;

/** function to run when the watch is handled */
typedef dbus_bool_t (* DBusWatchHandler) (DBusWatch    *watch,
                                          unsigned int  flags,
                                          void         *data);

DBusWatch* _dbus_watch_new                (int               fd,
                                           unsigned int      flags,
                                           dbus_bool_t       enabled,
                                           DBusWatchHandler  handler,
                                           void             *data,
                                           DBusFreeFunction  free_data_function);
DBusWatch* _dbus_watch_ref                (DBusWatch        *watch);
void       _dbus_watch_unref              (DBusWatch        *watch);
void       _dbus_watch_invalidate         (DBusWatch        *watch);
void       _dbus_watch_sanitize_condition (DBusWatch        *watch,
                                           unsigned int     *condition);
void       _dbus_watch_set_handler        (DBusWatch        *watch,
                                           DBusWatchHandler  handler,
                                           void             *data,
                                           DBusFreeFunction  free_data_function);


DBusWatchList* _dbus_watch_list_new           (void);
void           _dbus_watch_list_free          (DBusWatchList           *watch_list);
dbus_bool_t    _dbus_watch_list_set_functions (DBusWatchList           *watch_list,
                                               DBusAddWatchFunction     add_function,
                                               DBusRemoveWatchFunction  remove_function,
                                               DBusWatchToggledFunction toggled_function,
                                               void                    *data,
                                               DBusFreeFunction         free_data_function);
dbus_bool_t    _dbus_watch_list_add_watch     (DBusWatchList           *watch_list,
                                               DBusWatch               *watch);
void           _dbus_watch_list_remove_watch  (DBusWatchList           *watch_list,
                                               DBusWatch               *watch);
void           _dbus_watch_list_toggle_watch  (DBusWatchList           *watch_list,
                                               DBusWatch               *watch,
                                               dbus_bool_t              enabled);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_WATCH_H */
