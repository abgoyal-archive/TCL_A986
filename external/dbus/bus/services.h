
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_SERVICES_H
#define BUS_SERVICES_H

#include <dbus/dbus.h>
#include <dbus/dbus-string.h>
#include <dbus/dbus-hash.h>
#include "connection.h"
#include "bus.h"

typedef void (* BusServiceForeachFunction) (BusService       *service,
                                            void             *data);

BusRegistry* bus_registry_new             (BusContext                  *context);
BusRegistry* bus_registry_ref             (BusRegistry                 *registry);
void         bus_registry_unref           (BusRegistry                 *registry);
BusService*  bus_registry_lookup          (BusRegistry                 *registry,
                                           const DBusString            *service_name);
BusService*  bus_registry_ensure          (BusRegistry                 *registry,
                                           const DBusString            *service_name,
                                           DBusConnection              *owner_connection_if_created,
					   dbus_uint32_t                flags,
                                           BusTransaction              *transaction,
                                           DBusError                   *error);
void         bus_registry_foreach         (BusRegistry                 *registry,
                                           BusServiceForeachFunction    function,
                                           void                        *data);
dbus_bool_t  bus_registry_list_services   (BusRegistry                 *registry,
                                           char                      ***listp,
                                           int                         *array_len);
dbus_bool_t  bus_registry_acquire_service (BusRegistry                 *registry,
                                           DBusConnection              *connection,
                                           const DBusString            *service_name,
                                           dbus_uint32_t                flags,
                                           dbus_uint32_t               *result,
                                           BusTransaction              *transaction,
                                           DBusError                   *error);
dbus_bool_t  bus_registry_release_service (BusRegistry                 *registry,
                                           DBusConnection              *connection,
                                           const DBusString            *service_name,
                                           dbus_uint32_t               *result,
                                           BusTransaction              *transaction,
                                           DBusError                   *error);
dbus_bool_t  bus_registry_set_service_context_table (BusRegistry           *registry,
						     DBusHashTable         *table);

BusService*     bus_service_ref                       (BusService     *service);
void            bus_service_unref                     (BusService     *service);
dbus_bool_t     bus_service_add_owner                 (BusService     *service,
                                                       DBusConnection *connection,
						       dbus_uint32_t   flags,
                                                       BusTransaction *transaction,
                                                       DBusError      *error);
dbus_bool_t     bus_service_swap_owner                (BusService     *service,
                                                       DBusConnection *connection,
                                                       BusTransaction *transaction,
                                                       DBusError      *error);
dbus_bool_t     bus_service_remove_owner              (BusService     *service,
                                                       DBusConnection *connection,
                                                       BusTransaction *transaction,
                                                       DBusError      *error);
dbus_bool_t     bus_service_has_owner                 (BusService     *service,
                                                       DBusConnection *connection);
BusOwner*       bus_service_get_primary_owner         (BusService     *service);
dbus_bool_t     bus_service_get_allow_replacement     (BusService     *service);
const char*     bus_service_get_name                  (BusService     *service);
dbus_bool_t     bus_service_list_queued_owners        (BusService *service,
                                                       DBusList  **return_list,
                                                       DBusError  *error);

DBusConnection* bus_service_get_primary_owners_connection (BusService     *service);
#endif /* BUS_SERVICES_H */
