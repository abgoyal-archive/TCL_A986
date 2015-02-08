
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-connection-internal.h"
#include "dbus-pending-call-internal.h"
#include "dbus-pending-call.h"
#include "dbus-list.h"
#include "dbus-threads.h"
#include "dbus-test.h"



#define CONNECTION_LOCK(connection)   _dbus_connection_lock(connection)
#define CONNECTION_UNLOCK(connection) _dbus_connection_unlock(connection)

struct DBusPendingCall
{
  DBusAtomic refcount;                            /**< reference count */

  DBusDataSlotList slot_list;                     /**< Data stored by allocated integer ID */
  
  DBusPendingCallNotifyFunction function;         /**< Notifier when reply arrives. */

  DBusConnection *connection;                     /**< Connections we're associated with */
  DBusMessage *reply;                             /**< Reply (after we've received it) */
  DBusTimeout *timeout;                           /**< Timeout */

  DBusList *timeout_link;                         /**< Preallocated timeout response */
  
  dbus_uint32_t reply_serial;                     /**< Expected serial of reply */

  unsigned int completed : 1;                     /**< TRUE if completed */
  unsigned int timeout_added : 1;                 /**< Have added the timeout */
};

static dbus_int32_t notify_user_data_slot = -1;

DBusPendingCall*
_dbus_pending_call_new_unlocked (DBusConnection    *connection,
                                 int                timeout_milliseconds,
                                 DBusTimeoutHandler timeout_handler)
{
  DBusPendingCall *pending;
  DBusTimeout *timeout;

  _dbus_assert (timeout_milliseconds >= 0 || timeout_milliseconds == -1);
 
  if (timeout_milliseconds == -1)
    timeout_milliseconds = _DBUS_DEFAULT_TIMEOUT_VALUE;

  /* it would probably seem logical to pass in _DBUS_INT_MAX for
   * infinite timeout, but then math in
   * _dbus_connection_block_for_reply would get all overflow-prone, so
   * smack that down.
   */
  if (timeout_milliseconds > _DBUS_ONE_HOUR_IN_MILLISECONDS * 6)
    timeout_milliseconds = _DBUS_ONE_HOUR_IN_MILLISECONDS * 6;
  
  if (!dbus_pending_call_allocate_data_slot (&notify_user_data_slot))
    return NULL;
  
  pending = dbus_new0 (DBusPendingCall, 1);
  
  if (pending == NULL)
    {
      dbus_pending_call_free_data_slot (&notify_user_data_slot);
      return NULL;
    }

  timeout = _dbus_timeout_new (timeout_milliseconds,
                               timeout_handler,
			       pending, NULL);  

  if (timeout == NULL)
    {
      dbus_pending_call_free_data_slot (&notify_user_data_slot);
      dbus_free (pending);
      return NULL;
    }
  
  pending->refcount.value = 1;
  pending->connection = connection;
  _dbus_connection_ref_unlocked (pending->connection);

  pending->timeout = timeout;


  _dbus_data_slot_list_init (&pending->slot_list);
  
  return pending;
}

void
_dbus_pending_call_set_reply_unlocked (DBusPendingCall *pending,
                                       DBusMessage     *message)
{
  if (message == NULL)
    {
      message = pending->timeout_link->data;
      _dbus_list_clear (&pending->timeout_link);
    }
  else
    dbus_message_ref (message);

  _dbus_verbose ("  handing message %p (%s) to pending call serial %u\n",
                 message,
                 dbus_message_get_type (message) == DBUS_MESSAGE_TYPE_METHOD_RETURN ?
                 "method return" :
                 dbus_message_get_type (message) == DBUS_MESSAGE_TYPE_ERROR ?
                 "error" : "other type",
                 pending->reply_serial);
  
  _dbus_assert (pending->reply == NULL);
  _dbus_assert (pending->reply_serial == dbus_message_get_reply_serial (message));
  pending->reply = message;
}

void
_dbus_pending_call_complete (DBusPendingCall *pending)
{
  _dbus_assert (!pending->completed);
  
  pending->completed = TRUE;

  if (pending->function)
    {
      void *user_data;
      user_data = dbus_pending_call_get_data (pending,
                                              notify_user_data_slot);
      
      (* pending->function) (pending, user_data);
    }
}

void
_dbus_pending_call_queue_timeout_error_unlocked (DBusPendingCall *pending, 
                                                 DBusConnection  *connection)
{
  _dbus_assert (connection == pending->connection);
  
  if (pending->timeout_link)
    {
      _dbus_connection_queue_synthesized_message_link (connection,
						       pending->timeout_link);
      pending->timeout_link = NULL;
    }
}

dbus_bool_t 
_dbus_pending_call_is_timeout_added_unlocked (DBusPendingCall  *pending)
{
  _dbus_assert (pending != NULL);

  return pending->timeout_added;
}


void
_dbus_pending_call_set_timeout_added_unlocked (DBusPendingCall  *pending,
                                               dbus_bool_t       is_added)
{
  _dbus_assert (pending != NULL);

  pending->timeout_added = is_added;
}


DBusTimeout *
_dbus_pending_call_get_timeout_unlocked (DBusPendingCall  *pending)
{
  _dbus_assert (pending != NULL);

  return pending->timeout;
}

dbus_uint32_t 
_dbus_pending_call_get_reply_serial_unlocked (DBusPendingCall  *pending)
{
  _dbus_assert (pending != NULL);

  return pending->reply_serial;
}

void
_dbus_pending_call_set_reply_serial_unlocked  (DBusPendingCall *pending,
                                               dbus_uint32_t serial)
{
  _dbus_assert (pending != NULL);
  _dbus_assert (pending->reply_serial == 0);

  pending->reply_serial = serial;
}

DBusConnection *
_dbus_pending_call_get_connection_and_lock (DBusPendingCall *pending)
{
  _dbus_assert (pending != NULL);
 
  CONNECTION_LOCK (pending->connection);
  return pending->connection;
}

DBusConnection *
_dbus_pending_call_get_connection_unlocked (DBusPendingCall *pending)
{
  _dbus_assert (pending != NULL);
 
  return pending->connection;
}

dbus_bool_t
_dbus_pending_call_set_timeout_error_unlocked (DBusPendingCall *pending,
                                               DBusMessage     *message,
                                               dbus_uint32_t    serial)
{ 
  DBusList *reply_link;
  DBusMessage *reply;

  reply = dbus_message_new_error (message, DBUS_ERROR_NO_REPLY,
                                  "Did not receive a reply. Possible causes include: "
                                  "the remote application did not send a reply, "
                                  "the message bus security policy blocked the reply, "
                                  "the reply timeout expired, or "
                                  "the network connection was broken.");
  if (reply == NULL)
    return FALSE;

  reply_link = _dbus_list_alloc_link (reply);
  if (reply_link == NULL)
    {
      dbus_message_unref (reply);
      return FALSE;
    }

  pending->timeout_link = reply_link;

  _dbus_pending_call_set_reply_serial_unlocked (pending, serial);
  
  return TRUE;
}

DBusPendingCall *
_dbus_pending_call_ref_unlocked (DBusPendingCall *pending)
{
  pending->refcount.value += 1;
  
  return pending;
}


static void
_dbus_pending_call_last_unref (DBusPendingCall *pending)
{
  DBusConnection *connection;
  
  /* If we get here, we should be already detached
   * from the connection, or never attached.
   */
  _dbus_assert (!pending->timeout_added);  

  connection = pending->connection;

  /* this assumes we aren't holding connection lock... */
  _dbus_data_slot_list_free (&pending->slot_list);

  if (pending->timeout != NULL)
    _dbus_timeout_unref (pending->timeout);
      
  if (pending->timeout_link)
    {
      dbus_message_unref ((DBusMessage *)pending->timeout_link->data);
      _dbus_list_free_link (pending->timeout_link);
      pending->timeout_link = NULL;
    }

  if (pending->reply)
    {
      dbus_message_unref (pending->reply);
      pending->reply = NULL;
    }
      
  dbus_free (pending);

  dbus_pending_call_free_data_slot (&notify_user_data_slot);

  /* connection lock should not be held. */
  /* Free the connection last to avoid a weird state while
   * calling out to application code where the pending exists
   * but not the connection.
   */
  dbus_connection_unref (connection);
}

void
_dbus_pending_call_unref_and_unlock (DBusPendingCall *pending)
{
  dbus_bool_t last_unref;
  
  _dbus_assert (pending->refcount.value > 0);

  pending->refcount.value -= 1;
  last_unref = pending->refcount.value == 0;

  CONNECTION_UNLOCK (pending->connection);
  if (last_unref)
    _dbus_pending_call_last_unref (pending);
}

dbus_bool_t
_dbus_pending_call_get_completed_unlocked (DBusPendingCall    *pending)
{
  return pending->completed;
}

static DBusDataSlotAllocator slot_allocator;
_DBUS_DEFINE_GLOBAL_LOCK (pending_call_slots);

dbus_bool_t
_dbus_pending_call_set_data_unlocked (DBusPendingCall  *pending,
                                     dbus_int32_t      slot,
                                     void             *data,
                                     DBusFreeFunction  free_data_func)
{
  DBusFreeFunction old_free_func;
  void *old_data;
  dbus_bool_t retval;

  retval = _dbus_data_slot_list_set (&slot_allocator,
                                     &pending->slot_list,
                                     slot, data, free_data_func,
                                     &old_free_func, &old_data);

  /* Drop locks to call out to app code */
  CONNECTION_UNLOCK (pending->connection);
  
  if (retval)
    {
      if (old_free_func)
        (* old_free_func) (old_data);
    }

  CONNECTION_LOCK (pending->connection);
  
  return retval;
}

/** @} */



DBusPendingCall *
dbus_pending_call_ref (DBusPendingCall *pending)
{
  _dbus_return_val_if_fail (pending != NULL, NULL);

  /* The connection lock is better than the global
   * lock in the atomic increment fallback
   */
#ifdef DBUS_HAVE_ATOMIC_INT
  _dbus_atomic_inc (&pending->refcount);
#else
  CONNECTION_LOCK (pending->connection);
  _dbus_assert (pending->refcount.value > 0);

  pending->refcount.value += 1;
  CONNECTION_UNLOCK (pending->connection);
#endif
  
  return pending;
}

void
dbus_pending_call_unref (DBusPendingCall *pending)
{
  dbus_bool_t last_unref;

  _dbus_return_if_fail (pending != NULL);

  /* More efficient to use the connection lock instead of atomic
   * int fallback if we lack atomic int decrement
   */
#ifdef DBUS_HAVE_ATOMIC_INT
  last_unref = (_dbus_atomic_dec (&pending->refcount) == 1);
#else
  CONNECTION_LOCK (pending->connection);
  _dbus_assert (pending->refcount.value > 0);
  pending->refcount.value -= 1;
  last_unref = pending->refcount.value == 0;
  CONNECTION_UNLOCK (pending->connection);
#endif
  
  if (last_unref)
    _dbus_pending_call_last_unref(pending);
}

dbus_bool_t
dbus_pending_call_set_notify (DBusPendingCall              *pending,
                              DBusPendingCallNotifyFunction function,
                              void                         *user_data,
                              DBusFreeFunction              free_user_data)
{
  _dbus_return_val_if_fail (pending != NULL, FALSE);

  CONNECTION_LOCK (pending->connection);
  
  /* could invoke application code! */
  if (!_dbus_pending_call_set_data_unlocked (pending, notify_user_data_slot,
                                             user_data, free_user_data))
    return FALSE;
  
  pending->function = function;

  CONNECTION_UNLOCK (pending->connection);
  
  return TRUE;
}

void
dbus_pending_call_cancel (DBusPendingCall *pending)
{
  _dbus_return_if_fail (pending != NULL);

  _dbus_connection_remove_pending_call (pending->connection,
                                        pending);
}

dbus_bool_t
dbus_pending_call_get_completed (DBusPendingCall *pending)
{
  dbus_bool_t completed;
  
  _dbus_return_val_if_fail (pending != NULL, FALSE);

  CONNECTION_LOCK (pending->connection);
  completed = pending->completed;
  CONNECTION_UNLOCK (pending->connection);

  return completed;
}

DBusMessage*
dbus_pending_call_steal_reply (DBusPendingCall *pending)
{
  DBusMessage *message;
  
  _dbus_return_val_if_fail (pending != NULL, NULL);
  _dbus_return_val_if_fail (pending->completed, NULL);
  _dbus_return_val_if_fail (pending->reply != NULL, NULL);

  CONNECTION_LOCK (pending->connection);
  
  message = pending->reply;
  pending->reply = NULL;

  CONNECTION_UNLOCK (pending->connection);
  
  return message;
}

void
dbus_pending_call_block (DBusPendingCall *pending)
{
  _dbus_return_if_fail (pending != NULL);

  _dbus_connection_block_pending_call (pending);
}

dbus_bool_t
dbus_pending_call_allocate_data_slot (dbus_int32_t *slot_p)
{
  _dbus_return_val_if_fail (slot_p != NULL, FALSE);

  return _dbus_data_slot_allocator_alloc (&slot_allocator,
                                          &_DBUS_LOCK_NAME (pending_call_slots),
                                          slot_p);
}

void
dbus_pending_call_free_data_slot (dbus_int32_t *slot_p)
{
  _dbus_return_if_fail (slot_p != NULL);
  _dbus_return_if_fail (*slot_p >= 0);

  _dbus_data_slot_allocator_free (&slot_allocator, slot_p);
}

dbus_bool_t
dbus_pending_call_set_data (DBusPendingCall  *pending,
                            dbus_int32_t      slot,
                            void             *data,
                            DBusFreeFunction  free_data_func)
{
  dbus_bool_t retval;
  
  _dbus_return_val_if_fail (pending != NULL, FALSE);
  _dbus_return_val_if_fail (slot >= 0, FALSE);

  
  CONNECTION_LOCK (pending->connection);
  retval = _dbus_pending_call_set_data_unlocked (pending, slot, data, free_data_func);
  CONNECTION_UNLOCK (pending->connection);
  return retval;
}

void*
dbus_pending_call_get_data (DBusPendingCall   *pending,
                            dbus_int32_t       slot)
{
  void *res;

  _dbus_return_val_if_fail (pending != NULL, NULL);

  CONNECTION_LOCK (pending->connection);
  res = _dbus_data_slot_list_get (&slot_allocator,
                                  &pending->slot_list,
                                  slot);
  CONNECTION_UNLOCK (pending->connection);

  return res;
}

/** @} */

#ifdef DBUS_BUILD_TESTS

dbus_bool_t
_dbus_pending_call_test (const char *test_data_dir)
{  

  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */
