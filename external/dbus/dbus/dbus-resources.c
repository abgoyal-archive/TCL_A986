
/* -*- mode: C; c-file-style: "gnu" -*- */
#include <dbus/dbus-resources.h>
#include <dbus/dbus-internals.h>



struct DBusCounter
{
  int refcount;  /**< reference count */

  long value;    /**< current counter value */

  long notify_guard_value; /**< call notify function when crossing this value */
  DBusCounterNotifyFunction notify_function; /**< notify function */
  void *notify_data; /**< data for notify function */
};

/** @} */  /* end of resource limits internals docs */


DBusCounter*
_dbus_counter_new (void)
{
  DBusCounter *counter;

  counter = dbus_new (DBusCounter, 1);
  if (counter == NULL)
    return NULL;
  
  counter->refcount = 1;
  counter->value = 0;

  counter->notify_guard_value = 0;
  counter->notify_function = NULL;
  counter->notify_data = NULL;
  
  return counter;
}

DBusCounter *
_dbus_counter_ref (DBusCounter *counter)
{
  _dbus_assert (counter->refcount > 0);
  
  counter->refcount += 1;

  return counter;
}

void
_dbus_counter_unref (DBusCounter *counter)
{
  _dbus_assert (counter->refcount > 0);

  counter->refcount -= 1;

  if (counter->refcount == 0)
    {
      
      dbus_free (counter);
    }
}

void
_dbus_counter_adjust (DBusCounter *counter,
                      long         delta)
{
  long old = counter->value;
  
  counter->value += delta;

#if 0
  _dbus_verbose ("Adjusting counter %ld by %ld = %ld\n",
                 old, delta, counter->value);
#endif
  
  if (counter->notify_function != NULL &&
      ((old < counter->notify_guard_value &&
        counter->value >= counter->notify_guard_value) ||
       (old >= counter->notify_guard_value &&
        counter->value < counter->notify_guard_value)))
    (* counter->notify_function) (counter, counter->notify_data);
}

long
_dbus_counter_get_value (DBusCounter *counter)
{
  return counter->value;
}

void
_dbus_counter_set_notify (DBusCounter               *counter,
                          long                       guard_value,
                          DBusCounterNotifyFunction  function,
                          void                      *user_data)
{
  counter->notify_guard_value = guard_value;
  counter->notify_function = function;
  counter->notify_data = user_data;
}

/** @} */  /* end of resource limits exported API */
