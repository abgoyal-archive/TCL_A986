
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_SPAWN_H
#define DBUS_SPAWN_H

#include <dbus/dbus-string.h>
#include <dbus/dbus-errors.h>
#include <dbus/dbus-watch.h>

DBUS_BEGIN_DECLS

typedef void (* DBusSpawnChildSetupFunc) (void *user_data);

typedef struct DBusBabysitter DBusBabysitter;

dbus_bool_t _dbus_spawn_async_with_babysitter     (DBusBabysitter           **sitter_p,
                                                   char                     **argv,
                                                   DBusSpawnChildSetupFunc    child_setup,
                                                   void                      *user_data,
                                                   DBusError                 *error);
DBusBabysitter* _dbus_babysitter_ref              (DBusBabysitter            *sitter);
void        _dbus_babysitter_unref                (DBusBabysitter            *sitter);
void        _dbus_babysitter_kill_child           (DBusBabysitter            *sitter);
dbus_bool_t _dbus_babysitter_get_child_exited     (DBusBabysitter            *sitter);
void        _dbus_babysitter_set_child_exit_error (DBusBabysitter            *sitter,
                                                   DBusError                 *error);
dbus_bool_t _dbus_babysitter_set_watch_functions  (DBusBabysitter            *sitter,
                                                   DBusAddWatchFunction       add_function,
                                                   DBusRemoveWatchFunction    remove_function,
                                                   DBusWatchToggledFunction   toggled_function,
                                                   void                      *data,
                                                   DBusFreeFunction           free_data_function);

DBUS_END_DECLS

#endif /* DBUS_SPAWN_H */
