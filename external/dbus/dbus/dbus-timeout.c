
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-timeout.h"
#include "dbus-list.h"


struct DBusTimeout
{
  int refcount;                                /**< Reference count */
  int interval;                                /**< Timeout interval in milliseconds. */

  DBusTimeoutHandler handler;                  /**< Timeout handler. */
  void *handler_data;                          /**< Timeout handler data. */
  DBusFreeFunction free_handler_data_function; /**< Free the timeout handler data. */
  
  void *data;		   	               /**< Application data. */
  DBusFreeFunction free_data_function;         /**< Free the application data. */
  unsigned int enabled : 1;                    /**< True if timeout is active. */
};

DBusTimeout*
_dbus_timeout_new (int                 interval,
		   DBusTimeoutHandler  handler,
		   void               *data,
		   DBusFreeFunction    free_data_function)
{
  DBusTimeout *timeout;

  timeout = dbus_new0 (DBusTimeout, 1);
  if (timeout == NULL)
    return NULL;
  
  timeout->refcount = 1;
  timeout->interval = interval;

  timeout->handler = handler;
  timeout->handler_data = data;
  timeout->free_handler_data_function = free_data_function;

  timeout->enabled = TRUE;
  
  return timeout;
}

DBusTimeout *
_dbus_timeout_ref (DBusTimeout *timeout)
{
  timeout->refcount += 1;

  return timeout;
}

void
_dbus_timeout_unref (DBusTimeout *timeout)
{
  _dbus_assert (timeout != NULL);
  _dbus_assert (timeout->refcount > 0);
  
  timeout->refcount -= 1;
  if (timeout->refcount == 0)
    {
      dbus_timeout_set_data (timeout, NULL, NULL); /* call free_data_function */

      if (timeout->free_handler_data_function)
	(* timeout->free_handler_data_function) (timeout->handler_data);
      
      dbus_free (timeout);
    }
}

void
_dbus_timeout_set_interval (DBusTimeout *timeout,
                            int          interval)
{
  _dbus_assert (interval >= 0);
  
  timeout->interval = interval;
}

void
_dbus_timeout_set_enabled (DBusTimeout  *timeout,
                           dbus_bool_t   enabled)
{
  timeout->enabled = enabled != FALSE;
}



struct DBusTimeoutList
{
  DBusList *timeouts; /**< Timeout objects. */

  DBusAddTimeoutFunction add_timeout_function;       /**< Callback for adding a timeout. */
  DBusRemoveTimeoutFunction remove_timeout_function; /**< Callback for removing a timeout. */
  DBusTimeoutToggledFunction timeout_toggled_function; /**< Callback when timeout is enabled/disabled or changes interval */
  void *timeout_data;                                /**< Data for timeout callbacks */
  DBusFreeFunction timeout_free_data_function;       /**< Free function for timeout callback data */
};

DBusTimeoutList*
_dbus_timeout_list_new (void)
{
  DBusTimeoutList *timeout_list;

  timeout_list = dbus_new0 (DBusTimeoutList, 1);
  if (timeout_list == NULL)
    return NULL;

  return timeout_list;
}

void
_dbus_timeout_list_free (DBusTimeoutList *timeout_list)
{
  /* free timeout_data and remove timeouts as a side effect */
  _dbus_timeout_list_set_functions (timeout_list,
				    NULL, NULL, NULL, NULL, NULL);

  _dbus_list_foreach (&timeout_list->timeouts,
		      (DBusForeachFunction) _dbus_timeout_unref,
		      NULL);
  _dbus_list_clear (&timeout_list->timeouts);

  dbus_free (timeout_list);
}

dbus_bool_t
_dbus_timeout_list_set_functions (DBusTimeoutList           *timeout_list,
				  DBusAddTimeoutFunction     add_function,
				  DBusRemoveTimeoutFunction  remove_function,
                                  DBusTimeoutToggledFunction toggled_function,
				  void                      *data,
				  DBusFreeFunction           free_data_function)
{
  /* Add timeouts with the new function, failing on OOM */
  if (add_function != NULL)
    {
      DBusList *link;
      
      link = _dbus_list_get_first_link (&timeout_list->timeouts);
      while (link != NULL)
        {
          DBusList *next = _dbus_list_get_next_link (&timeout_list->timeouts,
                                                     link);
      
          if (!(* add_function) (link->data, data))
            {
              /* remove it all again and return FALSE */
              DBusList *link2;
              
              link2 = _dbus_list_get_first_link (&timeout_list->timeouts);
              while (link2 != link)
                {
                  DBusList *next = _dbus_list_get_next_link (&timeout_list->timeouts,
                                                             link2);

                  (* remove_function) (link2->data, data);
                  
                  link2 = next;
                }

              return FALSE;
            }
      
          link = next;
        }
    }
  
  /* Remove all current timeouts from previous timeout handlers */

  if (timeout_list->remove_timeout_function != NULL)
    {
      _dbus_list_foreach (&timeout_list->timeouts,
			  (DBusForeachFunction) timeout_list->remove_timeout_function,
			  timeout_list->timeout_data);
    }

  if (timeout_list->timeout_free_data_function != NULL)
    (* timeout_list->timeout_free_data_function) (timeout_list->timeout_data);

  timeout_list->add_timeout_function = add_function;
  timeout_list->remove_timeout_function = remove_function;
  timeout_list->timeout_toggled_function = toggled_function;
  timeout_list->timeout_data = data;
  timeout_list->timeout_free_data_function = free_data_function;

  return TRUE;
}

dbus_bool_t
_dbus_timeout_list_add_timeout (DBusTimeoutList *timeout_list,
				DBusTimeout     *timeout)
{
  if (!_dbus_list_append (&timeout_list->timeouts, timeout))
    return FALSE;

  _dbus_timeout_ref (timeout);

  if (timeout_list->add_timeout_function != NULL)
    {
      if (!(* timeout_list->add_timeout_function) (timeout,
                                                   timeout_list->timeout_data))
        {
          _dbus_list_remove_last (&timeout_list->timeouts, timeout);
          _dbus_timeout_unref (timeout);
          return FALSE;
        }
    }

  return TRUE;
}

void
_dbus_timeout_list_remove_timeout (DBusTimeoutList *timeout_list,
				   DBusTimeout     *timeout)
{
  if (!_dbus_list_remove (&timeout_list->timeouts, timeout))
    _dbus_assert_not_reached ("Nonexistent timeout was removed");

  if (timeout_list->remove_timeout_function != NULL)
    (* timeout_list->remove_timeout_function) (timeout,
					       timeout_list->timeout_data);

  _dbus_timeout_unref (timeout);
}

void
_dbus_timeout_list_toggle_timeout (DBusTimeoutList           *timeout_list,
                                   DBusTimeout               *timeout,
                                   dbus_bool_t                enabled)
{
  enabled = !!enabled;
  
  if (enabled == timeout->enabled)
    return;

  timeout->enabled = enabled;
  
  if (timeout_list->timeout_toggled_function != NULL)
    (* timeout_list->timeout_toggled_function) (timeout,
                                                timeout_list->timeout_data);
}

/** @} */




int
dbus_timeout_get_interval (DBusTimeout *timeout)
{
  return timeout->interval;
}

void*
dbus_timeout_get_data (DBusTimeout *timeout)
{
  return timeout->data;
}

void
dbus_timeout_set_data (DBusTimeout      *timeout,
		       void             *data,
		       DBusFreeFunction  free_data_function)
{
  if (timeout->free_data_function != NULL)
    (* timeout->free_data_function) (timeout->data);

  timeout->data = data;
  timeout->free_data_function = free_data_function;
}

dbus_bool_t
dbus_timeout_handle (DBusTimeout *timeout)
{
  return (* timeout->handler) (timeout->handler_data);
}


dbus_bool_t
dbus_timeout_get_enabled (DBusTimeout *timeout)
{
  return timeout->enabled;
}

/** @} end public API docs */
