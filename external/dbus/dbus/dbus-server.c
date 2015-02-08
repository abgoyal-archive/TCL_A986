
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-server.h"
#include "dbus-server-unix.h"
#include "dbus-server-socket.h"
#include "dbus-string.h"
#ifdef DBUS_BUILD_TESTS
#include "dbus-server-debug-pipe.h"
#endif
#include "dbus-address.h"
#include "dbus-protocol.h"



static char*
copy_address_with_guid_appended (const DBusString *address,
                                 const DBusString *guid_hex)
{
  DBusString with_guid;
  char *retval;
  
  if (!_dbus_string_init (&with_guid))
    return NULL;

  if (!_dbus_string_copy (address, 0, &with_guid,
                          _dbus_string_get_length (&with_guid)) ||
      !_dbus_string_append (&with_guid, ",guid=") ||
      !_dbus_string_copy (guid_hex, 0,
                          &with_guid, _dbus_string_get_length (&with_guid)))
    {
      _dbus_string_free (&with_guid);
      return NULL;
    }

  retval = NULL;
  _dbus_string_steal_data (&with_guid, &retval);

  _dbus_string_free (&with_guid);
      
  return retval; /* may be NULL if steal_data failed */
}

dbus_bool_t
_dbus_server_init_base (DBusServer             *server,
                        const DBusServerVTable *vtable,
                        const DBusString       *address)
{
  server->vtable = vtable;
  server->refcount.value = 1;

  server->address = NULL;
  server->watches = NULL;
  server->timeouts = NULL;

  if (!_dbus_string_init (&server->guid_hex))
    return FALSE;

  _dbus_generate_uuid (&server->guid);

  if (!_dbus_uuid_encode (&server->guid, &server->guid_hex))
    goto failed;
  
  server->address = copy_address_with_guid_appended (address,
                                                     &server->guid_hex);
  if (server->address == NULL)
    goto failed;
  
  _dbus_mutex_new_at_location (&server->mutex);
  if (server->mutex == NULL)
    goto failed;
  
  server->watches = _dbus_watch_list_new ();
  if (server->watches == NULL)
    goto failed;

  server->timeouts = _dbus_timeout_list_new ();
  if (server->timeouts == NULL)
    goto failed;

  _dbus_data_slot_list_init (&server->slot_list);

  _dbus_verbose ("Initialized server on address %s\n", server->address);
  
  return TRUE;

 failed:
  _dbus_mutex_free_at_location (&server->mutex);
  server->mutex = NULL;
  if (server->watches)
    {
      _dbus_watch_list_free (server->watches);
      server->watches = NULL;
    }
  if (server->timeouts)
    {
      _dbus_timeout_list_free (server->timeouts);
      server->timeouts = NULL;
    }
  if (server->address)
    {
      dbus_free (server->address);
      server->address = NULL;
    }
  _dbus_string_free (&server->guid_hex);
  
  return FALSE;
}

void
_dbus_server_finalize_base (DBusServer *server)
{
  /* We don't have the lock, but nobody should be accessing
   * concurrently since they don't have a ref
   */
#ifndef DBUS_DISABLE_CHECKS
  _dbus_assert (!server->have_server_lock);
#endif
  _dbus_assert (server->disconnected);
  
  /* calls out to application code... */
  _dbus_data_slot_list_free (&server->slot_list);

  dbus_server_set_new_connection_function (server, NULL, NULL, NULL);

  _dbus_watch_list_free (server->watches);
  _dbus_timeout_list_free (server->timeouts);

  _dbus_mutex_free_at_location (&server->mutex);
  
  dbus_free (server->address);

  dbus_free_string_array (server->auth_mechanisms);

  _dbus_string_free (&server->guid_hex);
}


/** Function to be called in protected_change_watch() with refcount held */
typedef dbus_bool_t (* DBusWatchAddFunction)     (DBusWatchList *list,
                                                  DBusWatch     *watch);
/** Function to be called in protected_change_watch() with refcount held */
typedef void        (* DBusWatchRemoveFunction)  (DBusWatchList *list,
                                                  DBusWatch     *watch);
/** Function to be called in protected_change_watch() with refcount held */
typedef void        (* DBusWatchToggleFunction)  (DBusWatchList *list,
                                                  DBusWatch     *watch,
                                                  dbus_bool_t    enabled);

static dbus_bool_t
protected_change_watch (DBusServer             *server,
                        DBusWatch              *watch,
                        DBusWatchAddFunction    add_function,
                        DBusWatchRemoveFunction remove_function,
                        DBusWatchToggleFunction toggle_function,
                        dbus_bool_t             enabled)
{
  DBusWatchList *watches;
  dbus_bool_t retval;
  
  HAVE_LOCK_CHECK (server);

  /* This isn't really safe or reasonable; a better pattern is the "do
   * everything, then drop lock and call out" one; but it has to be
   * propagated up through all callers
   */
  
  watches = server->watches;
  if (watches)
    {
      server->watches = NULL;
      _dbus_server_ref_unlocked (server);
      SERVER_UNLOCK (server);

      if (add_function)
        retval = (* add_function) (watches, watch);
      else if (remove_function)
        {
          retval = TRUE;
          (* remove_function) (watches, watch);
        }
      else
        {
          retval = TRUE;
          (* toggle_function) (watches, watch, enabled);
        }
      
      SERVER_LOCK (server);
      server->watches = watches;
      _dbus_server_unref_unlocked (server);

      return retval;
    }
  else
    return FALSE;
}

dbus_bool_t
_dbus_server_add_watch (DBusServer *server,
                        DBusWatch  *watch)
{
  HAVE_LOCK_CHECK (server);
  return protected_change_watch (server, watch,
                                 _dbus_watch_list_add_watch,
                                 NULL, NULL, FALSE);
}

void
_dbus_server_remove_watch  (DBusServer *server,
                            DBusWatch  *watch)
{
  HAVE_LOCK_CHECK (server);
  protected_change_watch (server, watch,
                          NULL,
                          _dbus_watch_list_remove_watch,
                          NULL, FALSE);
}

void
_dbus_server_toggle_watch (DBusServer  *server,
                           DBusWatch   *watch,
                           dbus_bool_t  enabled)
{
  _dbus_assert (watch != NULL);

  HAVE_LOCK_CHECK (server);
  protected_change_watch (server, watch,
                          NULL, NULL,
                          _dbus_watch_list_toggle_watch,
                          enabled);
}

/** Function to be called in protected_change_timeout() with refcount held */
typedef dbus_bool_t (* DBusTimeoutAddFunction)    (DBusTimeoutList *list,
                                                   DBusTimeout     *timeout);
/** Function to be called in protected_change_timeout() with refcount held */
typedef void        (* DBusTimeoutRemoveFunction) (DBusTimeoutList *list,
                                                   DBusTimeout     *timeout);
/** Function to be called in protected_change_timeout() with refcount held */
typedef void        (* DBusTimeoutToggleFunction) (DBusTimeoutList *list,
                                                   DBusTimeout     *timeout,
                                                   dbus_bool_t      enabled);


static dbus_bool_t
protected_change_timeout (DBusServer               *server,
                          DBusTimeout              *timeout,
                          DBusTimeoutAddFunction    add_function,
                          DBusTimeoutRemoveFunction remove_function,
                          DBusTimeoutToggleFunction toggle_function,
                          dbus_bool_t               enabled)
{
  DBusTimeoutList *timeouts;
  dbus_bool_t retval;
  
  HAVE_LOCK_CHECK (server);

  /* This isn't really safe or reasonable; a better pattern is the "do everything, then
   * drop lock and call out" one; but it has to be propagated up through all callers
   */
  
  timeouts = server->timeouts;
  if (timeouts)
    {
      server->timeouts = NULL;
      _dbus_server_ref_unlocked (server);
      SERVER_UNLOCK (server);

      if (add_function)
        retval = (* add_function) (timeouts, timeout);
      else if (remove_function)
        {
          retval = TRUE;
          (* remove_function) (timeouts, timeout);
        }
      else
        {
          retval = TRUE;
          (* toggle_function) (timeouts, timeout, enabled);
        }
      
      SERVER_LOCK (server);
      server->timeouts = timeouts;
      _dbus_server_unref_unlocked (server);

      return retval;
    }
  else
    return FALSE;
}

dbus_bool_t
_dbus_server_add_timeout (DBusServer  *server,
			  DBusTimeout *timeout)
{
  return protected_change_timeout (server, timeout,
                                   _dbus_timeout_list_add_timeout,
                                   NULL, NULL, FALSE);
}

void
_dbus_server_remove_timeout (DBusServer  *server,
			     DBusTimeout *timeout)
{
  protected_change_timeout (server, timeout,
                            NULL,
                            _dbus_timeout_list_remove_timeout,
                            NULL, FALSE);
}

void
_dbus_server_toggle_timeout (DBusServer  *server,
                             DBusTimeout *timeout,
                             dbus_bool_t  enabled)
{
  protected_change_timeout (server, timeout,
                            NULL, NULL,
                            _dbus_timeout_list_toggle_timeout,
                            enabled);
}


void
_dbus_server_ref_unlocked (DBusServer *server)
{
  _dbus_assert (server != NULL);
  _dbus_assert (server->refcount.value > 0);
  
  HAVE_LOCK_CHECK (server);

#ifdef DBUS_HAVE_ATOMIC_INT
  _dbus_atomic_inc (&server->refcount);
#else
  _dbus_assert (server->refcount.value > 0);

  server->refcount.value += 1;
#endif
}

void
_dbus_server_unref_unlocked (DBusServer *server)
{
  dbus_bool_t last_unref;

  /* Keep this in sync with dbus_server_unref */
  
  _dbus_assert (server != NULL);
  _dbus_assert (server->refcount.value > 0);

  HAVE_LOCK_CHECK (server);
  
#ifdef DBUS_HAVE_ATOMIC_INT
  last_unref = (_dbus_atomic_dec (&server->refcount) == 1);
#else
  _dbus_assert (server->refcount.value > 0);

  server->refcount.value -= 1;
  last_unref = (server->refcount.value == 0);
#endif
  
  if (last_unref)
    {
      _dbus_assert (server->disconnected);
      
      SERVER_UNLOCK (server);
      
      _dbus_assert (server->vtable->finalize != NULL);
      
      (* server->vtable->finalize) (server);
    }
}

/** @} */




static const struct {
  DBusServerListenResult (* func) (DBusAddressEntry *entry,
                                   DBusServer      **server_p,
                                   DBusError        *error);
} listen_funcs[] = {
  { _dbus_server_listen_socket },
  { _dbus_server_listen_platform_specific }
#ifdef DBUS_BUILD_TESTS
  , { _dbus_server_listen_debug_pipe }
#endif
};

DBusServer*
dbus_server_listen (const char     *address,
                    DBusError      *error)
{
  DBusServer *server;
  DBusAddressEntry **entries;
  int len, i;
  DBusError first_connect_error;
  dbus_bool_t handled_once;
  
  _dbus_return_val_if_fail (address != NULL, NULL);
  _dbus_return_val_if_error_is_set (error, NULL);
  
  if (!dbus_parse_address (address, &entries, &len, error))
    return NULL;

  server = NULL;
  dbus_error_init (&first_connect_error);
  handled_once = FALSE;
  
  for (i = 0; i < len; i++)
    {
      int j;

      for (j = 0; j < (int) _DBUS_N_ELEMENTS (listen_funcs); ++j)
        {
          DBusServerListenResult result;
          DBusError tmp_error;
      
          dbus_error_init (&tmp_error);
          result = (* listen_funcs[j].func) (entries[i],
                                             &server,
                                             &tmp_error);

          if (result == DBUS_SERVER_LISTEN_OK)
            {
              _dbus_assert (server != NULL);
              _DBUS_ASSERT_ERROR_IS_CLEAR (&tmp_error);
              handled_once = TRUE;
              goto out;
            }
          else if (result == DBUS_SERVER_LISTEN_BAD_ADDRESS)
            {
              _dbus_assert (server == NULL);
              _DBUS_ASSERT_ERROR_IS_SET (&tmp_error);
              dbus_move_error (&tmp_error, error);
              handled_once = TRUE;
              goto out;
            }
          else if (result == DBUS_SERVER_LISTEN_NOT_HANDLED)
            {
              _dbus_assert (server == NULL);
              _DBUS_ASSERT_ERROR_IS_CLEAR (&tmp_error);

              /* keep trying addresses */
            }
          else if (result == DBUS_SERVER_LISTEN_DID_NOT_CONNECT)
            {
              _dbus_assert (server == NULL);
              _DBUS_ASSERT_ERROR_IS_SET (&tmp_error);
              if (!dbus_error_is_set (&first_connect_error))
                dbus_move_error (&tmp_error, &first_connect_error);
              else
                dbus_error_free (&tmp_error);

              handled_once = TRUE;
              
              /* keep trying addresses */
            }
        }

      _dbus_assert (server == NULL);
      _DBUS_ASSERT_ERROR_IS_CLEAR (error);
    }

 out:

  if (!handled_once)
    {
      _DBUS_ASSERT_ERROR_IS_CLEAR (error);
      if (len > 0)
        dbus_set_error (error,
                       DBUS_ERROR_BAD_ADDRESS,
                       "Unknown address type '%s'",
                       dbus_address_entry_get_method (entries[0]));
      else
        dbus_set_error (error,
                        DBUS_ERROR_BAD_ADDRESS,
                        "Empty address '%s'",
                        address);
    }
  
  dbus_address_entries_free (entries);

  if (server == NULL)
    {
      _dbus_assert (error == NULL || dbus_error_is_set (&first_connect_error) ||
                   dbus_error_is_set (error));
      
      if (error && dbus_error_is_set (error))
        {
          /* already set the error */
        }
      else
        {
          /* didn't set the error but either error should be
           * NULL or first_connect_error should be set.
           */
          _dbus_assert (error == NULL || dbus_error_is_set (&first_connect_error));
          dbus_move_error (&first_connect_error, error);
        }

      _DBUS_ASSERT_ERROR_IS_CLEAR (&first_connect_error); /* be sure we freed it */
      _DBUS_ASSERT_ERROR_IS_SET (error);

      return NULL;
    }
  else
    {
      _DBUS_ASSERT_ERROR_IS_CLEAR (error);
      return server;
    }
}

DBusServer *
dbus_server_ref (DBusServer *server)
{
  _dbus_return_val_if_fail (server != NULL, NULL);
  _dbus_return_val_if_fail (server->refcount.value > 0, NULL);

#ifdef DBUS_HAVE_ATOMIC_INT
  _dbus_atomic_inc (&server->refcount);
#else
  SERVER_LOCK (server);
  _dbus_assert (server->refcount.value > 0);

  server->refcount.value += 1;
  SERVER_UNLOCK (server);
#endif

  return server;
}

void
dbus_server_unref (DBusServer *server)
{
  dbus_bool_t last_unref;

  /* keep this in sync with unref_unlocked */
  
  _dbus_return_if_fail (server != NULL);
  _dbus_return_if_fail (server->refcount.value > 0);

#ifdef DBUS_HAVE_ATOMIC_INT
  last_unref = (_dbus_atomic_dec (&server->refcount) == 1);
#else
  SERVER_LOCK (server);
  
  _dbus_assert (server->refcount.value > 0);

  server->refcount.value -= 1;
  last_unref = (server->refcount.value == 0);
  
  SERVER_UNLOCK (server);
#endif
  
  if (last_unref)
    {
      /* lock not held! */
      _dbus_assert (server->disconnected);
      
      _dbus_assert (server->vtable->finalize != NULL);
      
      (* server->vtable->finalize) (server);
    }
}

void
dbus_server_disconnect (DBusServer *server)
{
  _dbus_return_if_fail (server != NULL);
  _dbus_return_if_fail (server->refcount.value > 0);

  SERVER_LOCK (server);
  _dbus_server_ref_unlocked (server);
  
  _dbus_assert (server->vtable->disconnect != NULL);

  if (!server->disconnected)
    {
      /* this has to be first so recursive calls to disconnect don't happen */
      server->disconnected = TRUE;
      
      (* server->vtable->disconnect) (server);
    }

  SERVER_UNLOCK (server);
  dbus_server_unref (server);
}

dbus_bool_t
dbus_server_get_is_connected (DBusServer *server)
{
  dbus_bool_t retval;
  
  _dbus_return_val_if_fail (server != NULL, FALSE);

  SERVER_LOCK (server);
  retval = !server->disconnected;
  SERVER_UNLOCK (server);

  return retval;
}

char*
dbus_server_get_address (DBusServer *server)
{
  char *retval;
  
  _dbus_return_val_if_fail (server != NULL, NULL);

  SERVER_LOCK (server);
  retval = _dbus_strdup (server->address);
  SERVER_UNLOCK (server);

  return retval;
}

void
dbus_server_set_new_connection_function (DBusServer                *server,
                                         DBusNewConnectionFunction  function,
                                         void                      *data,
                                         DBusFreeFunction           free_data_function)
{
  DBusFreeFunction old_free_function;
  void *old_data;
  
  _dbus_return_if_fail (server != NULL);

  SERVER_LOCK (server);
  old_free_function = server->new_connection_free_data_function;
  old_data = server->new_connection_data;
  
  server->new_connection_function = function;
  server->new_connection_data = data;
  server->new_connection_free_data_function = free_data_function;
  SERVER_UNLOCK (server);
    
  if (old_free_function != NULL)
    (* old_free_function) (old_data);
}

dbus_bool_t
dbus_server_set_watch_functions (DBusServer              *server,
                                 DBusAddWatchFunction     add_function,
                                 DBusRemoveWatchFunction  remove_function,
                                 DBusWatchToggledFunction toggled_function,
                                 void                    *data,
                                 DBusFreeFunction         free_data_function)
{
  dbus_bool_t result;
  DBusWatchList *watches;
  
  _dbus_return_val_if_fail (server != NULL, FALSE);

  SERVER_LOCK (server);
  watches = server->watches;
  server->watches = NULL;
  if (watches)
    {
      SERVER_UNLOCK (server);
      result = _dbus_watch_list_set_functions (watches,
                                               add_function,
                                               remove_function,
                                               toggled_function,
                                               data,
                                               free_data_function);
      SERVER_LOCK (server);
    }
  else
    {
      _dbus_warn_check_failed ("Re-entrant call to %s\n", _DBUS_FUNCTION_NAME);
      result = FALSE;
    }
  server->watches = watches;
  SERVER_UNLOCK (server);
  
  return result;
}

dbus_bool_t
dbus_server_set_timeout_functions (DBusServer                *server,
				   DBusAddTimeoutFunction     add_function,
				   DBusRemoveTimeoutFunction  remove_function,
                                   DBusTimeoutToggledFunction toggled_function,
				   void                      *data,
				   DBusFreeFunction           free_data_function)
{
  dbus_bool_t result;
  DBusTimeoutList *timeouts;
  
  _dbus_return_val_if_fail (server != NULL, FALSE);

  SERVER_LOCK (server);
  timeouts = server->timeouts;
  server->timeouts = NULL;
  if (timeouts)
    {
      SERVER_UNLOCK (server);
      result = _dbus_timeout_list_set_functions (timeouts,
                                                 add_function,
                                                 remove_function,
                                                 toggled_function,
                                                 data,
                                                 free_data_function);
      SERVER_LOCK (server);
    }
  else
    {
      _dbus_warn_check_failed ("Re-entrant call to %s\n", _DBUS_FUNCTION_NAME);
      result = FALSE;
    }
  server->timeouts = timeouts;
  SERVER_UNLOCK (server);
  
  return result;
}

dbus_bool_t
dbus_server_set_auth_mechanisms (DBusServer  *server,
                                 const char **mechanisms)
{
  char **copy;

  _dbus_return_val_if_fail (server != NULL, FALSE);

  SERVER_LOCK (server);
  
  if (mechanisms != NULL)
    {
      copy = _dbus_dup_string_array (mechanisms);
      if (copy == NULL)
        return FALSE;
    }
  else
    copy = NULL;

  dbus_free_string_array (server->auth_mechanisms);
  server->auth_mechanisms = copy;

  SERVER_UNLOCK (server);
  
  return TRUE;
}


static DBusDataSlotAllocator slot_allocator;
_DBUS_DEFINE_GLOBAL_LOCK (server_slots);

dbus_bool_t
dbus_server_allocate_data_slot (dbus_int32_t *slot_p)
{
  return _dbus_data_slot_allocator_alloc (&slot_allocator,
                                          (DBusMutex **)&_DBUS_LOCK_NAME (server_slots),
                                          slot_p);
}

void
dbus_server_free_data_slot (dbus_int32_t *slot_p)
{
  _dbus_return_if_fail (*slot_p >= 0);
  
  _dbus_data_slot_allocator_free (&slot_allocator, slot_p);
}

dbus_bool_t
dbus_server_set_data (DBusServer       *server,
                      int               slot,
                      void             *data,
                      DBusFreeFunction  free_data_func)
{
  DBusFreeFunction old_free_func;
  void *old_data;
  dbus_bool_t retval;

  _dbus_return_val_if_fail (server != NULL, FALSE);

  SERVER_LOCK (server);
  
  retval = _dbus_data_slot_list_set (&slot_allocator,
                                     &server->slot_list,
                                     slot, data, free_data_func,
                                     &old_free_func, &old_data);


  SERVER_UNLOCK (server);
  
  if (retval)
    {
      /* Do the actual free outside the server lock */
      if (old_free_func)
        (* old_free_func) (old_data);
    }

  return retval;
}

void*
dbus_server_get_data (DBusServer   *server,
                      int           slot)
{
  void *res;

  _dbus_return_val_if_fail (server != NULL, NULL);
  
  SERVER_LOCK (server);
  
  res = _dbus_data_slot_list_get (&slot_allocator,
                                  &server->slot_list,
                                  slot);

  SERVER_UNLOCK (server);
  
  return res;
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include "dbus-test.h"

dbus_bool_t
_dbus_server_test (void)
{
  const char *valid_addresses[] = {
    "tcp:port=1234",
    "unix:path=./boogie",
    "tcp:host=localhost,port=1234",
    "tcp:host=localhost,port=1234;tcp:port=5678",
    "tcp:port=1234;unix:path=./boogie",
  };

  DBusServer *server;
  int i;
  
  for (i = 0; i < _DBUS_N_ELEMENTS (valid_addresses); i++)
    {
      DBusError error;

      /* FIXME um, how are the two tests here different? */
      
      dbus_error_init (&error);
      server = dbus_server_listen (valid_addresses[i], &error);
      if (server == NULL)
        {
          _dbus_warn ("server listen error: %s: %s\n", error.name, error.message);
          dbus_error_free (&error);
          _dbus_assert_not_reached ("Failed to listen for valid address.");
        }

      dbus_server_disconnect (server);
      dbus_server_unref (server);

      /* Try disconnecting before unreffing */
      server = dbus_server_listen (valid_addresses[i], &error);
      if (server == NULL)
        {
          _dbus_warn ("server listen error: %s: %s\n", error.name, error.message);
          dbus_error_free (&error);          
          _dbus_assert_not_reached ("Failed to listen for valid address.");
        }

      dbus_server_disconnect (server);
      dbus_server_unref (server);
    }

  return TRUE;
}

#endif /* DBUS_BUILD_TESTS */
