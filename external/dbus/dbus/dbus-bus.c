
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-bus.h"
#include "dbus-protocol.h"
#include "dbus-internals.h"
#include "dbus-message.h"
#include "dbus-marshal-validate.h"
#include "dbus-threads-internal.h"
#include "dbus-connection-internal.h"
#include <string.h>



typedef struct
{
  DBusConnection *connection; /**< Connection we're associated with */
  char *unique_name; /**< Unique name of this connection */

  unsigned int is_well_known : 1; /**< Is one of the well-known connections in our global array */
} BusData;

static dbus_int32_t bus_data_slot = -1;

/** Number of bus types */
#define N_BUS_TYPES 3

static DBusConnection *bus_connections[N_BUS_TYPES];
static char *bus_connection_addresses[N_BUS_TYPES] = { NULL, NULL, NULL };

static DBusBusType activation_bus_type = DBUS_BUS_STARTER;

static dbus_bool_t initialized = FALSE;

_DBUS_DEFINE_GLOBAL_LOCK (bus);

_DBUS_DEFINE_GLOBAL_LOCK (bus_datas);

static void
addresses_shutdown_func (void *data)
{
  int i;

  i = 0;
  while (i < N_BUS_TYPES)
    {
      if (bus_connections[i] != NULL)
        _dbus_warn_check_failed ("dbus_shutdown() called but connections were still live. This probably means the application did not drop all its references to bus connections.\n");
      
      dbus_free (bus_connection_addresses[i]);
      bus_connection_addresses[i] = NULL;
      ++i;
    }

  activation_bus_type = DBUS_BUS_STARTER;
}

static dbus_bool_t
get_from_env (char           **connection_p,
              const char      *env_var)
{
  const char *s;
  
  _dbus_assert (*connection_p == NULL);
  
  s = _dbus_getenv (env_var);
  if (s == NULL || *s == '\0')
    return TRUE; /* successfully didn't use the env var */
  else
    {
      *connection_p = _dbus_strdup (s);
      return *connection_p != NULL;
    }
}

static dbus_bool_t
init_connections_unlocked (void)
{
  if (!initialized)
    {
      const char *s;
      int i;

      i = 0;
      while (i < N_BUS_TYPES)
        {
          bus_connections[i] = NULL;
          ++i;
        }

      /* Don't init these twice, we may run this code twice if
       * init_connections_unlocked() fails midway through.
       * In practice, each block below should contain only one
       * "return FALSE" or running through twice may not
       * work right.
       */
      
       if (bus_connection_addresses[DBUS_BUS_SYSTEM] == NULL)
         {
           _dbus_verbose ("Filling in system bus address...\n");
           
           if (!get_from_env (&bus_connection_addresses[DBUS_BUS_SYSTEM],
                              "DBUS_SYSTEM_BUS_ADDRESS"))
             return FALSE;
         }

                  
       if (bus_connection_addresses[DBUS_BUS_SYSTEM] == NULL)
         {
           /* Use default system bus address if none set in environment */
           bus_connection_addresses[DBUS_BUS_SYSTEM] =
             _dbus_strdup (DBUS_SYSTEM_BUS_DEFAULT_ADDRESS);

           if (bus_connection_addresses[DBUS_BUS_SYSTEM] == NULL)
             return FALSE;
           
           _dbus_verbose ("  used default system bus \"%s\"\n",
                          bus_connection_addresses[DBUS_BUS_SYSTEM]);
         }
       else
         _dbus_verbose ("  used env var system bus \"%s\"\n",
                        bus_connection_addresses[DBUS_BUS_SYSTEM]);
          
      if (bus_connection_addresses[DBUS_BUS_SESSION] == NULL)
        {
          _dbus_verbose ("Filling in session bus address...\n");
          
          if (!get_from_env (&bus_connection_addresses[DBUS_BUS_SESSION],
                             "DBUS_SESSION_BUS_ADDRESS"))
            return FALSE;

	  if (bus_connection_addresses[DBUS_BUS_SESSION] == NULL)
	    bus_connection_addresses[DBUS_BUS_SESSION] =
	      _dbus_strdup (DBUS_SESSION_BUS_DEFAULT_ADDRESS);
          
          if (bus_connection_addresses[DBUS_BUS_SESSION] == NULL)
             return FALSE;

          _dbus_verbose ("  \"%s\"\n", bus_connection_addresses[DBUS_BUS_SESSION] ?
                         bus_connection_addresses[DBUS_BUS_SESSION] : "none set");
        }

      if (bus_connection_addresses[DBUS_BUS_STARTER] == NULL)
        {
          _dbus_verbose ("Filling in activation bus address...\n");
          
          if (!get_from_env (&bus_connection_addresses[DBUS_BUS_STARTER],
                             "DBUS_STARTER_ADDRESS"))
            return FALSE;
          
          _dbus_verbose ("  \"%s\"\n", bus_connection_addresses[DBUS_BUS_STARTER] ?
                         bus_connection_addresses[DBUS_BUS_STARTER] : "none set");
        }


      if (bus_connection_addresses[DBUS_BUS_STARTER] != NULL)
        {
          s = _dbus_getenv ("DBUS_STARTER_BUS_TYPE");
              
          if (s != NULL)
            {
              _dbus_verbose ("Bus activation type was set to \"%s\"\n", s);
                  
              if (strcmp (s, "system") == 0)
                activation_bus_type = DBUS_BUS_SYSTEM;
              else if (strcmp (s, "session") == 0)
                activation_bus_type = DBUS_BUS_SESSION;
            }
        }
      else
        {
          /* Default to the session bus instead if available */
          if (bus_connection_addresses[DBUS_BUS_SESSION] != NULL)
            {
              bus_connection_addresses[DBUS_BUS_STARTER] =
                _dbus_strdup (bus_connection_addresses[DBUS_BUS_SESSION]);
              if (bus_connection_addresses[DBUS_BUS_STARTER] == NULL)
                return FALSE;
            }
        }
      
      /* If we return FALSE we have to be sure that restarting
       * the above code will work right
       */
      
      if (!_dbus_setenv ("DBUS_ACTIVATION_ADDRESS", NULL))
        return FALSE;

      if (!_dbus_setenv ("DBUS_ACTIVATION_BUS_TYPE", NULL))
        return FALSE;
      
      if (!_dbus_register_shutdown_func (addresses_shutdown_func,
                                         NULL))
        return FALSE;
      
      initialized = TRUE;
    }

  return initialized;
}

static void
bus_data_free (void *data)
{
  BusData *bd = data;
  
  if (bd->is_well_known)
    {
      int i;
      _DBUS_LOCK (bus);
      /* We may be stored in more than one slot */
      /* This should now be impossible - these slots are supposed to
       * be cleared on disconnect, so should not need to be cleared on
       * finalize
       */
      i = 0;
      while (i < N_BUS_TYPES)
        {
          if (bus_connections[i] == bd->connection)
            bus_connections[i] = NULL;
          
          ++i;
        }
      _DBUS_UNLOCK (bus);
    }
  
  dbus_free (bd->unique_name);
  dbus_free (bd);

  dbus_connection_free_data_slot (&bus_data_slot);
}

static BusData*
ensure_bus_data (DBusConnection *connection)
{
  BusData *bd;

  if (!dbus_connection_allocate_data_slot (&bus_data_slot))
    return NULL;

  bd = dbus_connection_get_data (connection, bus_data_slot);
  if (bd == NULL)
    {      
      bd = dbus_new0 (BusData, 1);
      if (bd == NULL)
        {
          dbus_connection_free_data_slot (&bus_data_slot);
          return NULL;
        }

      bd->connection = connection;
      
      if (!dbus_connection_set_data (connection, bus_data_slot, bd,
                                     bus_data_free))
        {
          dbus_free (bd);
          dbus_connection_free_data_slot (&bus_data_slot);
          return NULL;
        }

      /* Data slot refcount now held by the BusData */
    }
  else
    {
      dbus_connection_free_data_slot (&bus_data_slot);
    }

  return bd;
}

void
_dbus_bus_notify_shared_connection_disconnected_unlocked (DBusConnection *connection)
{
  int i;
  
  _DBUS_LOCK (bus);

  /* We are expecting to have the connection saved in only one of these
   * slots, but someone could in a pathological case set system and session
   * bus to the same bus or something. Or set one of them to the starter
   * bus without setting the starter bus type in the env variable.
   * So we don't break the loop as soon as we find a match.
   */
  for (i = 0; i < N_BUS_TYPES; ++i)
    {
      if (bus_connections[i] == connection)
        {
          bus_connections[i] = NULL;
        }
    }

  _DBUS_UNLOCK (bus);
}

static DBusConnection *
internal_bus_get (DBusBusType  type,
                  dbus_bool_t  private,
                  DBusError   *error)
{
  const char *address;
  DBusConnection *connection;
  BusData *bd;
  DBusBusType address_type;

  _dbus_return_val_if_fail (type >= 0 && type < N_BUS_TYPES, NULL);
  _dbus_return_val_if_error_is_set (error, NULL);

  _DBUS_LOCK (bus);

  if (!init_connections_unlocked ())
    {
      _DBUS_UNLOCK (bus);
      _DBUS_SET_OOM (error);
      return NULL;
    }

  /* We want to use the activation address even if the
   * activating bus is the session or system bus,
   * per the spec.
   */
  address_type = type;
  
  /* Use the real type of the activation bus for getting its
   * connection, but only if the real type's address is available. (If
   * the activating bus isn't a well-known bus then
   * activation_bus_type == DBUS_BUS_STARTER)
   */
  if (type == DBUS_BUS_STARTER &&
      bus_connection_addresses[activation_bus_type] != NULL)
    type = activation_bus_type;
  
  if (!private && bus_connections[type] != NULL)
    {
      connection = bus_connections[type];
      dbus_connection_ref (connection);
      
      _DBUS_UNLOCK (bus);
      return connection;
    }

  address = bus_connection_addresses[address_type];
  if (address == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_FAILED,
                      "Unable to determine the address of the message bus (try 'man dbus-launch' and 'man dbus-daemon' for help)");
      _DBUS_UNLOCK (bus);
      return NULL;
    }

  if (private)
    connection = dbus_connection_open_private (address, error);
  else
    connection = dbus_connection_open (address, error);
  
  if (!connection)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      _DBUS_UNLOCK (bus);
      return NULL;
    }

  /* By default we're bound to the lifecycle of
   * the message bus.
   */
  dbus_connection_set_exit_on_disconnect (connection,
                                          TRUE);
  
  if (!dbus_bus_register (connection, error))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      _dbus_connection_close_possibly_shared (connection);
      dbus_connection_unref (connection);

      _DBUS_UNLOCK (bus);
      return NULL;
    }

  if (!private)
    {
      /* store a weak ref to the connection (dbus-connection.c is
       * supposed to have a strong ref that it drops on disconnect,
       * since this is a shared connection)
       */
      bus_connections[type] = connection;
    }

  _DBUS_LOCK (bus_datas);
  bd = ensure_bus_data (connection);
  _dbus_assert (bd != NULL); /* it should have been created on
                                register, so OOM not possible */
  bd->is_well_known = TRUE;
  _DBUS_UNLOCK (bus_datas);

  
  _DBUS_UNLOCK (bus);

  /* Return a reference to the caller */
  return connection;
}


/** @} */ /* end of implementation details docs */


DBusConnection *
dbus_bus_get (DBusBusType  type,
	      DBusError   *error)
{
  return internal_bus_get (type, FALSE, error);
}

DBusConnection *
dbus_bus_get_private (DBusBusType  type,
                      DBusError   *error)
{
  return internal_bus_get (type, TRUE, error);
}

dbus_bool_t
dbus_bus_register (DBusConnection *connection,
                   DBusError      *error)
{
  DBusMessage *message, *reply;
  char *name;
  BusData *bd;
  dbus_bool_t retval;

  _dbus_return_val_if_fail (connection != NULL, FALSE);
  _dbus_return_val_if_error_is_set (error, FALSE);

  retval = FALSE;

  _DBUS_LOCK (bus_datas);

  bd = ensure_bus_data (connection);
  if (bd == NULL)
    {
      _DBUS_SET_OOM (error);
      _DBUS_UNLOCK (bus_datas);
      return FALSE;
    }

  if (bd->unique_name != NULL)
    {
      _dbus_verbose ("Ignoring attempt to register the same DBusConnection %s with the message bus a second time.\n",
                     bd->unique_name);
      _DBUS_UNLOCK (bus_datas);

      /* Success! */
      return TRUE;
    }
  
  message = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                          DBUS_PATH_DBUS,
                                          DBUS_INTERFACE_DBUS,
                                          "Hello"); 

  if (!message)
    {
      _DBUS_SET_OOM (error);

      _DBUS_UNLOCK (bus_datas);
      return FALSE;
    }
  
  reply = dbus_connection_send_with_reply_and_block (connection, message, -1, error);

  dbus_message_unref (message);
  
  if (reply == NULL)
    goto out;
  else if (dbus_set_error_from_message (error, reply))
    goto out;
  else if (!dbus_message_get_args (reply, error,
                                   DBUS_TYPE_STRING, &name,
                                   DBUS_TYPE_INVALID))
    goto out;
  
  bd->unique_name = _dbus_strdup (name);
  if (bd->unique_name == NULL)
    {
      _DBUS_SET_OOM (error);
      goto out;
    }
  
  retval = TRUE;
  
 out:
  if (reply)
    dbus_message_unref (reply);

  if (!retval)
    _DBUS_ASSERT_ERROR_IS_SET (error);

  _DBUS_UNLOCK (bus_datas);
  
  return retval;
}


dbus_bool_t
dbus_bus_set_unique_name (DBusConnection *connection,
                          const char     *unique_name)
{
  BusData *bd;
  dbus_bool_t success;

  _dbus_return_val_if_fail (connection != NULL, FALSE);
  _dbus_return_val_if_fail (unique_name != NULL, FALSE);

  _DBUS_LOCK (bus_datas);
  
  bd = ensure_bus_data (connection);
  if (bd == NULL)
    return FALSE;

  _dbus_assert (bd->unique_name == NULL);
  
  bd->unique_name = _dbus_strdup (unique_name);
  success = bd->unique_name != NULL;
  
  _DBUS_UNLOCK (bus_datas);
  
  return success;
}

const char*
dbus_bus_get_unique_name (DBusConnection *connection)
{
  BusData *bd;
  const char *unique_name;

  _dbus_return_val_if_fail (connection != NULL, NULL);

  _DBUS_LOCK (bus_datas);
  
  bd = ensure_bus_data (connection);
  if (bd == NULL)
    return NULL;

  unique_name = bd->unique_name;

  _DBUS_UNLOCK (bus_datas);
  
  return unique_name;
}

unsigned long
dbus_bus_get_unix_user (DBusConnection *connection,
                        const char     *name,
                        DBusError      *error)
{
  DBusMessage *message, *reply;
  dbus_uint32_t uid;

  _dbus_return_val_if_fail (connection != NULL, DBUS_UID_UNSET);
  _dbus_return_val_if_fail (name != NULL, DBUS_UID_UNSET);
  _dbus_return_val_if_fail (_dbus_check_is_valid_bus_name (name), DBUS_UID_UNSET);
  _dbus_return_val_if_error_is_set (error, DBUS_UID_UNSET);
  
  message = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                          DBUS_PATH_DBUS,
                                          DBUS_INTERFACE_DBUS,
                                          "GetConnectionUnixUser");

  if (message == NULL)
    {
      _DBUS_SET_OOM (error);
      return DBUS_UID_UNSET;
    }
 
  if (!dbus_message_append_args (message,
				 DBUS_TYPE_STRING, &name,
				 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (message);
      _DBUS_SET_OOM (error);
      return DBUS_UID_UNSET;
    }
  
  reply = dbus_connection_send_with_reply_and_block (connection, message, -1,
                                                     error);
  
  dbus_message_unref (message);
  
  if (reply == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return DBUS_UID_UNSET;
    }  

  if (dbus_set_error_from_message (error, reply))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return DBUS_UID_UNSET;
    }
  
  if (!dbus_message_get_args (reply, error,
                              DBUS_TYPE_UINT32, &uid,
                              DBUS_TYPE_INVALID))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return DBUS_UID_UNSET;
    }

  dbus_message_unref (reply);
  
  return (unsigned long) uid;
}


int
dbus_bus_request_name (DBusConnection *connection,
                       const char     *name,
                       unsigned int    flags,
                       DBusError      *error)
{
  DBusMessage *message, *reply;
  dbus_uint32_t result;

  _dbus_return_val_if_fail (connection != NULL, 0);
  _dbus_return_val_if_fail (name != NULL, 0);
  _dbus_return_val_if_fail (_dbus_check_is_valid_bus_name (name), 0);
  _dbus_return_val_if_error_is_set (error, 0);
  
  message = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                          DBUS_PATH_DBUS,
                                          DBUS_INTERFACE_DBUS,
                                          "RequestName");

  if (message == NULL)
    {
      _DBUS_SET_OOM (error);
      return -1;
    }
 
  if (!dbus_message_append_args (message,
				 DBUS_TYPE_STRING, &name,
				 DBUS_TYPE_UINT32, &flags,
				 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (message);
      _DBUS_SET_OOM (error);
      return -1;
    }
  
  reply = dbus_connection_send_with_reply_and_block (connection, message, -1,
                                                     error);
  
  dbus_message_unref (message);
  
  if (reply == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return -1;
    }  

  if (dbus_set_error_from_message (error, reply))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return -1;
    }
  
  if (!dbus_message_get_args (reply, error,
                              DBUS_TYPE_UINT32, &result,
                              DBUS_TYPE_INVALID))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return -1;
    }

  dbus_message_unref (reply);
  
  return result;
}


int
dbus_bus_release_name (DBusConnection *connection,
                       const char     *name,
                       DBusError      *error)
{
  DBusMessage *message, *reply;
  dbus_uint32_t result;

  _dbus_return_val_if_fail (connection != NULL, 0);
  _dbus_return_val_if_fail (name != NULL, 0);
  _dbus_return_val_if_fail (_dbus_check_is_valid_bus_name (name), 0);
  _dbus_return_val_if_error_is_set (error, 0);

  message = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                          DBUS_PATH_DBUS,
                                          DBUS_INTERFACE_DBUS,
                                          "ReleaseName");

  if (message == NULL)
    {
      _DBUS_SET_OOM (error);
      return -1;
    }

  if (!dbus_message_append_args (message,
                                 DBUS_TYPE_STRING, &name,
                                 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (message);
      _DBUS_SET_OOM (error);
      return -1;
    }

  reply = dbus_connection_send_with_reply_and_block (connection, message, -1,
                                                     error);

  dbus_message_unref (message);

  if (reply == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return -1;
    }

  if (dbus_set_error_from_message (error, reply))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return -1;
    }

  if (!dbus_message_get_args (reply, error,
                              DBUS_TYPE_UINT32, &result,
                              DBUS_TYPE_INVALID))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return -1;
    }

  dbus_message_unref (reply);

  return result;
}

dbus_bool_t
dbus_bus_name_has_owner (DBusConnection *connection,
			 const char     *name,
                         DBusError      *error)
{
  DBusMessage *message, *reply;
  dbus_bool_t exists;

  _dbus_return_val_if_fail (connection != NULL, FALSE);
  _dbus_return_val_if_fail (name != NULL, FALSE);
  _dbus_return_val_if_fail (_dbus_check_is_valid_bus_name (name), FALSE);
  _dbus_return_val_if_error_is_set (error, FALSE);
  
  message = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                          DBUS_PATH_DBUS,
                                          DBUS_INTERFACE_DBUS,
                                          "NameHasOwner");
  if (message == NULL)
    {
      _DBUS_SET_OOM (error);
      return FALSE;
    }
  
  if (!dbus_message_append_args (message,
				 DBUS_TYPE_STRING, &name,
				 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (message);
      _DBUS_SET_OOM (error);
      return FALSE;
    }
  
  reply = dbus_connection_send_with_reply_and_block (connection, message, -1, error);
  dbus_message_unref (message);

  if (reply == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return FALSE;
    }

  if (!dbus_message_get_args (reply, error,
                              DBUS_TYPE_BOOLEAN, &exists,
                              DBUS_TYPE_INVALID))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return FALSE;
    }
  
  dbus_message_unref (reply);
  return exists;
}

dbus_bool_t
dbus_bus_start_service_by_name (DBusConnection *connection,
                                const char     *name,
                                dbus_uint32_t   flags,
                                dbus_uint32_t  *result,
                                DBusError      *error)
{
  DBusMessage *msg;
  DBusMessage *reply;

  _dbus_return_val_if_fail (connection != NULL, FALSE);
  _dbus_return_val_if_fail (_dbus_check_is_valid_bus_name (name), FALSE);
  
  msg = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                      DBUS_PATH_DBUS,
                                      DBUS_INTERFACE_DBUS,
                                      "StartServiceByName");

  if (!dbus_message_append_args (msg, DBUS_TYPE_STRING, &name,
			  	 DBUS_TYPE_UINT32, &flags, DBUS_TYPE_INVALID))
    {
      dbus_message_unref (msg);
      _DBUS_SET_OOM (error);
      return FALSE;
    }

  reply = dbus_connection_send_with_reply_and_block (connection, msg,
                                                     -1, error);
  dbus_message_unref (msg);

  if (reply == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return FALSE;
    }

  if (dbus_set_error_from_message (error, reply))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return FALSE;
    }

  if (result != NULL &&
      !dbus_message_get_args (reply, error, DBUS_TYPE_UINT32,
	      		      result, DBUS_TYPE_INVALID))
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      dbus_message_unref (reply);
      return FALSE;
    }
  
  dbus_message_unref (reply);
  return TRUE;
}

static void
send_no_return_values (DBusConnection *connection,
                       DBusMessage    *msg,
                       DBusError      *error)
{
  if (error)
    {
      /* Block to check success codepath */
      DBusMessage *reply;
      
      reply = dbus_connection_send_with_reply_and_block (connection, msg,
                                                         -1, error);
      
      if (reply == NULL)
        _DBUS_ASSERT_ERROR_IS_SET (error);
      else
        dbus_message_unref (reply);
    }
  else
    {
      /* Silently-fail nonblocking codepath */
      dbus_message_set_no_reply (msg, TRUE);
      dbus_connection_send (connection, msg, NULL);
    }
}

void
dbus_bus_add_match (DBusConnection *connection,
                    const char     *rule,
                    DBusError      *error)
{
  DBusMessage *msg;

  _dbus_return_if_fail (rule != NULL);

  msg = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                      DBUS_PATH_DBUS,
                                      DBUS_INTERFACE_DBUS,
                                      "AddMatch");

  if (msg == NULL)
    {
      _DBUS_SET_OOM (error);
      return;
    }

  if (!dbus_message_append_args (msg, DBUS_TYPE_STRING, &rule,
                                 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (msg);
      _DBUS_SET_OOM (error);
      return;
    }

  send_no_return_values (connection, msg, error);

  dbus_message_unref (msg);
}

void
dbus_bus_remove_match (DBusConnection *connection,
                       const char     *rule,
                       DBusError      *error)
{
  DBusMessage *msg;

  _dbus_return_if_fail (rule != NULL);
  
  msg = dbus_message_new_method_call (DBUS_SERVICE_DBUS,
                                      DBUS_PATH_DBUS,
                                      DBUS_INTERFACE_DBUS,
                                      "RemoveMatch");

  if (!dbus_message_append_args (msg, DBUS_TYPE_STRING, &rule,
                                 DBUS_TYPE_INVALID))
    {
      dbus_message_unref (msg);
      _DBUS_SET_OOM (error);
      return;
    }

  send_no_return_values (connection, msg, error);

  dbus_message_unref (msg);
}

/** @} */
