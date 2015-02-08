
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_BUS_H
#define BUS_BUS_H

#include <config.h>

#include <dbus/dbus.h>
#include <dbus/dbus-string.h>
#include <dbus/dbus-mainloop.h>
#include <dbus/dbus-userdb.h>

typedef struct BusActivation    BusActivation;
typedef struct BusConnections   BusConnections;
typedef struct BusContext       BusContext;
typedef struct BusPolicy        BusPolicy;
typedef struct BusClientPolicy  BusClientPolicy;
typedef struct BusPolicyRule    BusPolicyRule;
typedef struct BusRegistry      BusRegistry;
typedef struct BusSELinuxID     BusSELinuxID;
typedef struct BusService       BusService;
typedef struct BusOwner		BusOwner;
typedef struct BusTransaction   BusTransaction;
typedef struct BusMatchmaker    BusMatchmaker;
typedef struct BusMatchRule     BusMatchRule;

typedef struct
{
  long max_incoming_bytes;          /**< How many incoming message bytes for a single connection */
  long max_outgoing_bytes;          /**< How many outgoing bytes can be queued for a single connection */
  long max_message_size;            /**< Max size of a single message in bytes */
  int activation_timeout;           /**< How long to wait for an activation to time out */
  int auth_timeout;                 /**< How long to wait for an authentication to time out */
  int max_completed_connections;    /**< Max number of authorized connections */
  int max_incomplete_connections;   /**< Max number of incomplete connections */
  int max_connections_per_user;     /**< Max number of connections auth'd as same user */
  int max_pending_activations;      /**< Max number of pending activations for the entire bus */
  int max_services_per_connection;  /**< Max number of owned services for a single connection */
  int max_match_rules_per_connection; /**< Max number of match rules for a single connection */
  int max_replies_per_connection;     /**< Max number of replies that can be pending for each connection */
  int reply_timeout;                  /**< How long to wait before timing out a reply */
} BusLimits;

typedef enum
{
  FORK_FOLLOW_CONFIG_FILE,
  FORK_ALWAYS,
  FORK_NEVER
} ForceForkSetting;

BusContext*       bus_context_new                                (const DBusString *config_file,
                                                                  ForceForkSetting  force_fork,
                                                                  int               print_addr_fd,
                                                                  int               print_pid_fd,
                                                                  DBusError        *error);
dbus_bool_t       bus_context_reload_config                      (BusContext       *context,
								  DBusError        *error);
void              bus_context_shutdown                           (BusContext       *context);
BusContext*       bus_context_ref                                (BusContext       *context);
void              bus_context_unref                              (BusContext       *context);
const char*       bus_context_get_type                           (BusContext       *context);
const char*       bus_context_get_address                        (BusContext       *context);
BusRegistry*      bus_context_get_registry                       (BusContext       *context);
BusConnections*   bus_context_get_connections                    (BusContext       *context);
BusActivation*    bus_context_get_activation                     (BusContext       *context);
BusMatchmaker*    bus_context_get_matchmaker                     (BusContext       *context);
DBusLoop*         bus_context_get_loop                           (BusContext       *context);
DBusUserDatabase* bus_context_get_user_database                  (BusContext       *context);

dbus_bool_t       bus_context_allow_user                         (BusContext       *context,
                                                                  unsigned long     uid);
BusPolicy*        bus_context_get_policy                         (BusContext       *context);

BusClientPolicy*  bus_context_create_client_policy               (BusContext       *context,
                                                                  DBusConnection   *connection,
                                                                  DBusError        *error);
int               bus_context_get_activation_timeout             (BusContext       *context);
int               bus_context_get_auth_timeout                   (BusContext       *context);
int               bus_context_get_max_completed_connections      (BusContext       *context);
int               bus_context_get_max_incomplete_connections     (BusContext       *context);
int               bus_context_get_max_connections_per_user       (BusContext       *context);
int               bus_context_get_max_pending_activations        (BusContext       *context);
int               bus_context_get_max_services_per_connection    (BusContext       *context);
int               bus_context_get_max_match_rules_per_connection (BusContext       *context);
int               bus_context_get_max_replies_per_connection     (BusContext       *context);
int               bus_context_get_reply_timeout                  (BusContext       *context);
dbus_bool_t       bus_context_check_security_policy              (BusContext       *context,
                                                                  BusTransaction   *transaction,
                                                                  DBusConnection   *sender,
                                                                  DBusConnection   *addressed_recipient,
                                                                  DBusConnection   *proposed_recipient,
                                                                  DBusMessage      *message,
                                                                  DBusError        *error);

#endif /* BUS_BUS_H */
