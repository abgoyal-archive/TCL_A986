
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-watch.h"
#include "dbus-list.h"


struct DBusWatch
{
  int refcount;                        /**< Reference count */
  int fd;                              /**< File descriptor. */
  unsigned int flags;                  /**< Conditions to watch. */

  DBusWatchHandler handler;                    /**< Watch handler. */
  void *handler_data;                          /**< Watch handler data. */
  DBusFreeFunction free_handler_data_function; /**< Free the watch handler data. */
  
  void *data;                          /**< Application data. */
  DBusFreeFunction free_data_function; /**< Free the application data. */
  unsigned int enabled : 1;            /**< Whether it's enabled. */
};

DBusWatch*
_dbus_watch_new (int               fd,
                 unsigned int      flags,
                 dbus_bool_t       enabled,
                 DBusWatchHandler  handler,
                 void             *data,
                 DBusFreeFunction  free_data_function)
{
  DBusWatch *watch;

#define VALID_WATCH_FLAGS (DBUS_WATCH_WRITABLE | DBUS_WATCH_READABLE)
  
  _dbus_assert ((flags & VALID_WATCH_FLAGS) == flags);
  
  watch = dbus_new0 (DBusWatch, 1);
  if (watch == NULL)
    return NULL;
  
  watch->refcount = 1;
  watch->fd = fd;
  watch->flags = flags;
  watch->enabled = enabled;

  watch->handler = handler;
  watch->handler_data = data;
  watch->free_handler_data_function = free_data_function;
  
  return watch;
}

DBusWatch *
_dbus_watch_ref (DBusWatch *watch)
{
  watch->refcount += 1;

  return watch;
}

void
_dbus_watch_unref (DBusWatch *watch)
{
  _dbus_assert (watch != NULL);
  _dbus_assert (watch->refcount > 0);

  watch->refcount -= 1;
  if (watch->refcount == 0)
    {
      dbus_watch_set_data (watch, NULL, NULL); /* call free_data_function */

      if (watch->free_handler_data_function)
	(* watch->free_handler_data_function) (watch->handler_data);
      
      dbus_free (watch);
    }
}

void
_dbus_watch_invalidate (DBusWatch *watch)
{
  watch->fd = -1;
  watch->flags = 0;
}

void
_dbus_watch_sanitize_condition (DBusWatch    *watch,
                                unsigned int *condition)
{
  if (!(watch->flags & DBUS_WATCH_READABLE))
    *condition &= ~DBUS_WATCH_READABLE;
  if (!(watch->flags & DBUS_WATCH_WRITABLE))
    *condition &= ~DBUS_WATCH_WRITABLE;
}



struct DBusWatchList
{
  DBusList *watches;           /**< Watch objects. */

  DBusAddWatchFunction add_watch_function;    /**< Callback for adding a watch. */
  DBusRemoveWatchFunction remove_watch_function; /**< Callback for removing a watch. */
  DBusWatchToggledFunction watch_toggled_function; /**< Callback on toggling enablement */
  void *watch_data;                           /**< Data for watch callbacks */
  DBusFreeFunction watch_free_data_function;  /**< Free function for watch callback data */
};

DBusWatchList*
_dbus_watch_list_new (void)
{
  DBusWatchList *watch_list;

  watch_list = dbus_new0 (DBusWatchList, 1);
  if (watch_list == NULL)
    return NULL;

  return watch_list;
}

void
_dbus_watch_list_free (DBusWatchList *watch_list)
{
  /* free watch_data and removes watches as a side effect */
  _dbus_watch_list_set_functions (watch_list,
                                  NULL, NULL, NULL, NULL, NULL);
  _dbus_list_foreach (&watch_list->watches,
                      (DBusForeachFunction) _dbus_watch_unref,
                      NULL);
  _dbus_list_clear (&watch_list->watches);

  dbus_free (watch_list);
}

dbus_bool_t
_dbus_watch_list_set_functions (DBusWatchList           *watch_list,
                                DBusAddWatchFunction     add_function,
                                DBusRemoveWatchFunction  remove_function,
                                DBusWatchToggledFunction toggled_function,
                                void                    *data,
                                DBusFreeFunction         free_data_function)
{
  /* Add watches with the new watch function, failing on OOM */
  if (add_function != NULL)
    {
      DBusList *link;
      
      link = _dbus_list_get_first_link (&watch_list->watches);
      while (link != NULL)
        {
          DBusList *next = _dbus_list_get_next_link (&watch_list->watches,
                                                     link);

#ifdef DBUS_ENABLE_VERBOSE_MODE
          {
            const char *watch_type;
            int flags;

            flags = dbus_watch_get_flags (link->data);
            if ((flags & DBUS_WATCH_READABLE) &&
                (flags & DBUS_WATCH_WRITABLE))
              watch_type = "readwrite";
            else if (flags & DBUS_WATCH_READABLE)
              watch_type = "read";
            else if (flags & DBUS_WATCH_WRITABLE)
              watch_type = "write";
            else
              watch_type = "not read or write";
            
            _dbus_verbose ("Adding a %s watch on fd %d using newly-set add watch function\n",
                           watch_type,
                           dbus_watch_get_fd (link->data));
          }
#endif /* DBUS_ENABLE_VERBOSE_MODE */
          
          if (!(* add_function) (link->data, data))
            {
              /* remove it all again and return FALSE */
              DBusList *link2;
              
              link2 = _dbus_list_get_first_link (&watch_list->watches);
              while (link2 != link)
                {
                  DBusList *next = _dbus_list_get_next_link (&watch_list->watches,
                                                             link2);
                  
                  _dbus_verbose ("Removing watch on fd %d using newly-set remove function because initial add failed\n",
                                 dbus_watch_get_fd (link2->data));
                  
                  (* remove_function) (link2->data, data);
                  
                  link2 = next;
                }

              return FALSE;
            }
      
          link = next;
        }
    }
  
  /* Remove all current watches from previous watch handlers */

  if (watch_list->remove_watch_function != NULL)
    {
      _dbus_verbose ("Removing all pre-existing watches\n");
      
      _dbus_list_foreach (&watch_list->watches,
                          (DBusForeachFunction) watch_list->remove_watch_function,
                          watch_list->watch_data);
    }

  if (watch_list->watch_free_data_function != NULL)
    (* watch_list->watch_free_data_function) (watch_list->watch_data);
  
  watch_list->add_watch_function = add_function;
  watch_list->remove_watch_function = remove_function;
  watch_list->watch_toggled_function = toggled_function;
  watch_list->watch_data = data;
  watch_list->watch_free_data_function = free_data_function;

  return TRUE;
}

dbus_bool_t
_dbus_watch_list_add_watch (DBusWatchList *watch_list,
                            DBusWatch     *watch)
{
  if (!_dbus_list_append (&watch_list->watches, watch))
    return FALSE;
  
  _dbus_watch_ref (watch);

  if (watch_list->add_watch_function != NULL)
    {
      _dbus_verbose ("Adding watch on fd %d\n",
                     dbus_watch_get_fd (watch));
      
      if (!(* watch_list->add_watch_function) (watch,
                                               watch_list->watch_data))
        {
          _dbus_list_remove_last (&watch_list->watches, watch);
          _dbus_watch_unref (watch);
          return FALSE;
        }
    }
  
  return TRUE;
}

void
_dbus_watch_list_remove_watch  (DBusWatchList *watch_list,
                                DBusWatch     *watch)
{
  if (!_dbus_list_remove (&watch_list->watches, watch))
    _dbus_assert_not_reached ("Nonexistent watch was removed");
  
  if (watch_list->remove_watch_function != NULL)
    {
      _dbus_verbose ("Removing watch on fd %d\n",
                     dbus_watch_get_fd (watch));
      
      (* watch_list->remove_watch_function) (watch,
                                             watch_list->watch_data);
    }
  
  _dbus_watch_unref (watch);
}

void
_dbus_watch_list_toggle_watch (DBusWatchList           *watch_list,
                               DBusWatch               *watch,
                               dbus_bool_t              enabled)
{
  enabled = !!enabled;
  
  if (enabled == watch->enabled)
    return;

  watch->enabled = enabled;
  
  if (watch_list->watch_toggled_function != NULL)
    {
      _dbus_verbose ("Toggling watch %p on fd %d to %d\n",
                     watch, dbus_watch_get_fd (watch), watch->enabled);
      
      (* watch_list->watch_toggled_function) (watch,
                                              watch_list->watch_data);
    }
}

void
_dbus_watch_set_handler (DBusWatch        *watch,
                         DBusWatchHandler  handler,
                         void             *data,
                         DBusFreeFunction  free_data_function)
{
  if (watch->free_handler_data_function)
    (* watch->free_handler_data_function) (watch->handler_data);

  watch->handler = handler;
  watch->handler_data = data;
  watch->free_handler_data_function = free_data_function;
}

/** @} */



int
dbus_watch_get_fd (DBusWatch *watch)
{
  return watch->fd;
}

unsigned int
dbus_watch_get_flags (DBusWatch *watch)
{
  _dbus_assert ((watch->flags & VALID_WATCH_FLAGS) == watch->flags);

  return watch->flags;
}

void*
dbus_watch_get_data (DBusWatch *watch)
{
  return watch->data;
}

void
dbus_watch_set_data (DBusWatch        *watch,
                     void             *data,
                     DBusFreeFunction  free_data_function)
{
  _dbus_verbose ("Setting watch fd %d data to data = %p function = %p from data = %p function = %p\n",
                 dbus_watch_get_fd (watch),
                 data, free_data_function, watch->data, watch->free_data_function);
  
  if (watch->free_data_function != NULL)
    (* watch->free_data_function) (watch->data);
  
  watch->data = data;
  watch->free_data_function = free_data_function;
}

dbus_bool_t
dbus_watch_get_enabled (DBusWatch *watch)
{
  _dbus_assert (watch != NULL);
  return watch->enabled;
}


dbus_bool_t
dbus_watch_handle (DBusWatch    *watch,
                   unsigned int  flags)
{
#ifndef DBUS_DISABLE_CHECKS
  if (watch->fd < 0 || watch->flags == 0)
    {
      _dbus_warn_check_failed ("%s: Watch is invalid, it should have been removed\n",
                               _DBUS_FUNCTION_NAME);
      return TRUE;
    }
#endif
    
  _dbus_return_val_if_fail (watch->fd >= 0 /* fails if watch was removed */, TRUE);
  
  _dbus_watch_sanitize_condition (watch, &flags);

  if (flags == 0)
    {
      _dbus_verbose ("After sanitization, watch flags on fd %d were 0\n",
                     watch->fd);
      return TRUE;
    }
  else
    return (* watch->handler) (watch, flags,
                               watch->handler_data);
}


/** @} */
