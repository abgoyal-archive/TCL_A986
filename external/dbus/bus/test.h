
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_TEST_H
#define BUS_TEST_H

#include <config.h>

#ifdef DBUS_BUILD_TESTS

#include <dbus/dbus.h>
#include <dbus/dbus-string.h>
#include "connection.h"

dbus_bool_t bus_dispatch_test         (const DBusString             *test_data_dir);
dbus_bool_t bus_dispatch_sha1_test    (const DBusString             *test_data_dir);
dbus_bool_t bus_policy_test           (const DBusString             *test_data_dir);
dbus_bool_t bus_config_parser_test    (const DBusString             *test_data_dir);
dbus_bool_t bus_signals_test          (const DBusString             *test_data_dir);
dbus_bool_t bus_expire_list_test      (const DBusString             *test_data_dir);
dbus_bool_t bus_activation_service_reload_test (const DBusString    *test_data_dir);
dbus_bool_t bus_setup_debug_client    (DBusConnection               *connection);
void        bus_test_clients_foreach  (BusConnectionForeachFunction  function,
                                       void                         *data);
dbus_bool_t bus_test_client_listed    (DBusConnection               *connection);
void        bus_test_run_bus_loop     (BusContext                   *context,
                                       dbus_bool_t                   block);
void        bus_test_run_clients_loop (dbus_bool_t                   block);
void        bus_test_run_everything   (BusContext                   *context);
BusContext* bus_context_new_test      (const DBusString             *test_data_dir,
                                       const char                   *filename);



#endif

#endif /* BUS_TEST_H */
