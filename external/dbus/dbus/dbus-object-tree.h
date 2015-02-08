
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_OBJECT_TREE_H
#define DBUS_OBJECT_TREE_H

#include <dbus/dbus-connection.h>

DBUS_BEGIN_DECLS

typedef struct DBusObjectTree DBusObjectTree;

DBusObjectTree* _dbus_object_tree_new   (DBusConnection *connection);
DBusObjectTree* _dbus_object_tree_ref   (DBusObjectTree *tree);
void            _dbus_object_tree_unref (DBusObjectTree *tree);

dbus_bool_t       _dbus_object_tree_register               (DBusObjectTree              *tree,
                                                            dbus_bool_t                  fallback,
                                                            const char                 **path,
                                                            const DBusObjectPathVTable  *vtable,
                                                            void                        *user_data);
void              _dbus_object_tree_unregister_and_unlock  (DBusObjectTree              *tree,
                                                            const char                 **path);
DBusHandlerResult _dbus_object_tree_dispatch_and_unlock    (DBusObjectTree              *tree,
                                                            DBusMessage                 *message);
void*             _dbus_object_tree_get_user_data_unlocked (DBusObjectTree              *tree,
                                                            const char                 **path);
void              _dbus_object_tree_free_all_unlocked      (DBusObjectTree              *tree);


dbus_bool_t _dbus_object_tree_list_registered_and_unlock (DBusObjectTree *tree,
                                                          const char    **parent_path,
                                                          char         ***child_entries);

dbus_bool_t _dbus_decompose_path (const char   *data,
                                  int           len,
                                  char       ***path,
                                  int          *path_len);

DBUS_END_DECLS

#endif /* DBUS_OBJECT_TREE_H */
