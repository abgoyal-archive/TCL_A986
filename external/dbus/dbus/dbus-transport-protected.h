
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_TRANSPORT_PROTECTED_H
#define DBUS_TRANSPORT_PROTECTED_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-errors.h>
#include <dbus/dbus-transport.h>
#include <dbus/dbus-message-internal.h>
#include <dbus/dbus-auth.h>
#include <dbus/dbus-resources.h>

DBUS_BEGIN_DECLS

typedef struct DBusTransportVTable DBusTransportVTable;

struct DBusTransportVTable
{
  void        (* finalize)              (DBusTransport *transport);
  /**< The finalize method must free the transport. */

  dbus_bool_t (* handle_watch)          (DBusTransport *transport,
                                         DBusWatch     *watch,
                                         unsigned int   flags);
  /**< The handle_watch method handles reading/writing
   * data as indicated by the flags.
   */

  void        (* disconnect)            (DBusTransport *transport);
  /**< Disconnect this transport. */

  dbus_bool_t (* connection_set)        (DBusTransport *transport);
  /**< Called when transport->connection has been filled in */

  void        (* do_iteration)          (DBusTransport *transport,
                                         unsigned int   flags,
                                         int            timeout_milliseconds);
  /**< Called to do a single "iteration" (block on select/poll
   * followed by reading or writing data).
   */

  void        (* live_messages_changed) (DBusTransport *transport);
  /**< Outstanding messages counter changed */

  dbus_bool_t (* get_socket_fd) (DBusTransport *transport,
                                 int           *fd_p);
  /**< Get socket file descriptor */
};

struct DBusTransport
{
  int refcount;                               /**< Reference count. */

  const DBusTransportVTable *vtable;          /**< Virtual methods for this instance. */

  DBusConnection *connection;                 /**< Connection owning this transport. */

  DBusMessageLoader *loader;                  /**< Message-loading buffer. */

  DBusAuth *auth;                             /**< Authentication conversation */

  DBusCredentials credentials;                /**< Credentials of other end */  

  long max_live_messages_size;                /**< Max total size of received messages. */

  DBusCounter *live_messages_size;            /**< Counter for size of all live messages. */


  char *address;                              /**< Address of the server we are connecting to (#NULL for the server side of a transport) */

  char *expected_guid;                        /**< GUID we expect the server to have, #NULL on server side or if we don't have an expectation */
  
  DBusAllowUnixUserFunction unix_user_function; /**< Function for checking whether a user is authorized. */
  void *unix_user_data;                         /**< Data for unix_user_function */
  
  DBusFreeFunction free_unix_user_data;         /**< Function to free unix_user_data */
  
  unsigned int disconnected : 1;              /**< #TRUE if we are disconnected. */
  unsigned int authenticated : 1;             /**< Cache of auth state; use _dbus_transport_get_is_authenticated() to query value */
  unsigned int send_credentials_pending : 1;  /**< #TRUE if we need to send credentials */
  unsigned int receive_credentials_pending : 1; /**< #TRUE if we need to receive credentials */
  unsigned int is_server : 1;                 /**< #TRUE if on the server side */
  unsigned int unused_bytes_recovered : 1;    /**< #TRUE if we've recovered unused bytes from auth */
};

dbus_bool_t _dbus_transport_init_base     (DBusTransport             *transport,
                                           const DBusTransportVTable *vtable,
                                           const DBusString          *server_guid,
                                           const DBusString          *address);
void        _dbus_transport_finalize_base (DBusTransport             *transport);


typedef enum
{
  DBUS_TRANSPORT_OPEN_NOT_HANDLED,    /**< we aren't in charge of this address type */
  DBUS_TRANSPORT_OPEN_OK,             /**< we set up the listen */
  DBUS_TRANSPORT_OPEN_BAD_ADDRESS,    /**< malformed address */
  DBUS_TRANSPORT_OPEN_DID_NOT_CONNECT /**< well-formed address but failed to set it up */
} DBusTransportOpenResult;

DBusTransportOpenResult _dbus_transport_open_platform_specific (DBusAddressEntry  *entry,
                                                                DBusTransport    **transport_p,
                                                                DBusError         *error);

DBUS_END_DECLS

#endif /* DBUS_TRANSPORT_PROTECTED_H */
