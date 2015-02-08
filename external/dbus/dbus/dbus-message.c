
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-marshal-recursive.h"
#include "dbus-marshal-validate.h"
#include "dbus-marshal-byteswap.h"
#include "dbus-marshal-header.h"
#include "dbus-signature.h"
#include "dbus-message-private.h"
#include "dbus-object-tree.h"
#include "dbus-memory.h"
#include "dbus-list.h"
#include "dbus-threads-internal.h"
#include <string.h>

static void dbus_message_finalize (DBusMessage *message);


/** An static string representing an empty signature */
_DBUS_STRING_DEFINE_STATIC(_dbus_empty_signature_str,  "");

enum {
  DBUS_MESSAGE_ITER_TYPE_READER = 3,
  DBUS_MESSAGE_ITER_TYPE_WRITER = 7
};

/** typedef for internals of message iterator */
typedef struct DBusMessageRealIter DBusMessageRealIter;

struct DBusMessageRealIter
{
  DBusMessage *message; /**< Message used */
  dbus_uint32_t changed_stamp : CHANGED_STAMP_BITS; /**< stamp to detect invalid iters */
  dbus_uint32_t iter_type : 3;      /**< whether this is a reader or writer iter */
  dbus_uint32_t sig_refcount : 8;   /**< depth of open_signature() */
  union
  {
    DBusTypeWriter writer; /**< writer */
    DBusTypeReader reader; /**< reader */
  } u; /**< the type writer or reader that does all the work */
};

static void
get_const_signature (DBusHeader        *header,
                     const DBusString **type_str_p,
                     int               *type_pos_p)
{
  if (_dbus_header_get_field_raw (header,
                                  DBUS_HEADER_FIELD_SIGNATURE,
                                  type_str_p,
                                  type_pos_p))
    {
      *type_pos_p += 1; /* skip the signature length which is 1 byte */
    }
  else
    {
      *type_str_p = &_dbus_empty_signature_str;
      *type_pos_p = 0;
    }
}

static void
_dbus_message_byteswap (DBusMessage *message)
{
  const DBusString *type_str;
  int type_pos;
  
  if (message->byte_order == DBUS_COMPILER_BYTE_ORDER)
    return;

  _dbus_verbose ("Swapping message into compiler byte order\n");
  
  get_const_signature (&message->header, &type_str, &type_pos);
  
  _dbus_marshal_byteswap (type_str, type_pos,
                          message->byte_order,
                          DBUS_COMPILER_BYTE_ORDER,
                          &message->body, 0);

  message->byte_order = DBUS_COMPILER_BYTE_ORDER;
  
  _dbus_header_byteswap (&message->header, DBUS_COMPILER_BYTE_ORDER);
}

#define ensure_byte_order(message)                      \
 if (message->byte_order != DBUS_COMPILER_BYTE_ORDER)   \
   _dbus_message_byteswap (message)

void
_dbus_message_get_network_data (DBusMessage          *message,
                                const DBusString    **header,
                                const DBusString    **body)
{
  _dbus_assert (message->locked);

  *header = &message->header.data;
  *body = &message->body;
}

void
_dbus_message_set_serial (DBusMessage   *message,
                          dbus_uint32_t  serial)
{
  _dbus_assert (message != NULL);
  _dbus_assert (!message->locked);
  _dbus_assert (dbus_message_get_serial (message) == 0);

  _dbus_header_set_serial (&message->header, serial);
}

void
_dbus_message_add_size_counter_link (DBusMessage  *message,
                                     DBusList     *link)
{
  /* right now we don't recompute the delta when message
   * size changes, and that's OK for current purposes
   * I think, but could be important to change later.
   * Do recompute it whenever there are no outstanding counters,
   * since it's basically free.
   */
  if (message->size_counters == NULL)
    {
      message->size_counter_delta =
        _dbus_string_get_length (&message->header.data) +
        _dbus_string_get_length (&message->body);

#if 0
      _dbus_verbose ("message has size %ld\n",
                     message->size_counter_delta);
#endif
    }

  _dbus_list_append_link (&message->size_counters, link);

  _dbus_counter_adjust (link->data, message->size_counter_delta);
}

dbus_bool_t
_dbus_message_add_size_counter (DBusMessage *message,
                                DBusCounter *counter)
{
  DBusList *link;

  link = _dbus_list_alloc_link (counter);
  if (link == NULL)
    return FALSE;

  _dbus_counter_ref (counter);
  _dbus_message_add_size_counter_link (message, link);

  return TRUE;
}

void
_dbus_message_remove_size_counter (DBusMessage  *message,
                                   DBusCounter  *counter,
                                   DBusList    **link_return)
{
  DBusList *link;

  link = _dbus_list_find_last (&message->size_counters,
                               counter);
  _dbus_assert (link != NULL);

  _dbus_list_unlink (&message->size_counters,
                     link);
  if (link_return)
    *link_return = link;
  else
    _dbus_list_free_link (link);

  _dbus_counter_adjust (counter, - message->size_counter_delta);

  _dbus_counter_unref (counter);
}

void
_dbus_message_lock (DBusMessage  *message)
{
  if (!message->locked)
    {
      _dbus_header_update_lengths (&message->header,
                                   _dbus_string_get_length (&message->body));

      /* must have a signature if you have a body */
      _dbus_assert (_dbus_string_get_length (&message->body) == 0 ||
                    dbus_message_get_signature (message) != NULL);

      message->locked = TRUE;
    }
}

static dbus_bool_t
set_or_delete_string_field (DBusMessage *message,
                            int          field,
                            int          typecode,
                            const char  *value)
{
  if (value == NULL)
    return _dbus_header_delete_field (&message->header, field);
  else
    return _dbus_header_set_field_basic (&message->header,
                                         field,
                                         typecode,
                                         &value);
}

#if 0
/* Probably we don't need to use this */
static dbus_bool_t
_dbus_message_set_signature (DBusMessage *message,
                             const char  *signature)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (signature == NULL ||
                            _dbus_check_is_valid_signature (signature));
  /* can't delete the signature if you have a message body */
  _dbus_return_val_if_fail (_dbus_string_get_length (&message->body) == 0 ||
                            signature != NULL);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_SIGNATURE,
                                     DBUS_TYPE_SIGNATURE,
                                     signature);
}
#endif


/** Avoid caching huge messages */
#define MAX_MESSAGE_SIZE_TO_CACHE 10 * _DBUS_ONE_KILOBYTE

/** Avoid caching too many messages */
#define MAX_MESSAGE_CACHE_SIZE    5

_DBUS_DEFINE_GLOBAL_LOCK (message_cache);
static DBusMessage *message_cache[MAX_MESSAGE_CACHE_SIZE];
static int message_cache_count = 0;
static dbus_bool_t message_cache_shutdown_registered = FALSE;

static void
dbus_message_cache_shutdown (void *data)
{
  int i;

  _DBUS_LOCK (message_cache);

  i = 0;
  while (i < MAX_MESSAGE_CACHE_SIZE)
    {
      if (message_cache[i])
        dbus_message_finalize (message_cache[i]);

      ++i;
    }

  message_cache_count = 0;
  message_cache_shutdown_registered = FALSE;

  _DBUS_UNLOCK (message_cache);
}

static DBusMessage*
dbus_message_get_cached (void)
{
  DBusMessage *message;
  int i;

  message = NULL;

  _DBUS_LOCK (message_cache);

  _dbus_assert (message_cache_count >= 0);

  if (message_cache_count == 0)
    {
      _DBUS_UNLOCK (message_cache);
      return NULL;
    }

  /* This is not necessarily true unless count > 0, and
   * message_cache is uninitialized until the shutdown is
   * registered
   */
  _dbus_assert (message_cache_shutdown_registered);

  i = 0;
  while (i < MAX_MESSAGE_CACHE_SIZE)
    {
      if (message_cache[i])
        {
          message = message_cache[i];
          message_cache[i] = NULL;
          message_cache_count -= 1;
          break;
        }
      ++i;
    }
  _dbus_assert (message_cache_count >= 0);
  _dbus_assert (i < MAX_MESSAGE_CACHE_SIZE);
  _dbus_assert (message != NULL);

  _DBUS_UNLOCK (message_cache);

  _dbus_assert (message->refcount.value == 0);
  _dbus_assert (message->size_counters == NULL);

  return message;
}

static void
free_size_counter (void *element,
                   void *data)
{
  DBusCounter *counter = element;
  DBusMessage *message = data;

  _dbus_counter_adjust (counter, - message->size_counter_delta);

  _dbus_counter_unref (counter);
}

static void
dbus_message_cache_or_finalize (DBusMessage *message)
{
  dbus_bool_t was_cached;
  int i;
  
  _dbus_assert (message->refcount.value == 0);

  /* This calls application code and has to be done first thing
   * without holding the lock
   */
  _dbus_data_slot_list_clear (&message->slot_list);

  _dbus_list_foreach (&message->size_counters,
                      free_size_counter, message);
  _dbus_list_clear (&message->size_counters);

  was_cached = FALSE;

  _DBUS_LOCK (message_cache);

  if (!message_cache_shutdown_registered)
    {
      _dbus_assert (message_cache_count == 0);

      if (!_dbus_register_shutdown_func (dbus_message_cache_shutdown, NULL))
        goto out;

      i = 0;
      while (i < MAX_MESSAGE_CACHE_SIZE)
        {
          message_cache[i] = NULL;
          ++i;
        }

      message_cache_shutdown_registered = TRUE;
    }

  _dbus_assert (message_cache_count >= 0);

  if ((_dbus_string_get_length (&message->header.data) +
       _dbus_string_get_length (&message->body)) >
      MAX_MESSAGE_SIZE_TO_CACHE)
    goto out;

  if (message_cache_count >= MAX_MESSAGE_CACHE_SIZE)
    goto out;

  /* Find empty slot */
  i = 0;
  while (message_cache[i] != NULL)
    ++i;

  _dbus_assert (i < MAX_MESSAGE_CACHE_SIZE);

  _dbus_assert (message_cache[i] == NULL);
  message_cache[i] = message;
  message_cache_count += 1;
  was_cached = TRUE;
#ifndef DBUS_DISABLE_CHECKS
  message->in_cache = TRUE;
#endif

 out:
  _DBUS_UNLOCK (message_cache);

  _dbus_assert (message->refcount.value == 0);
  
  if (!was_cached)
    dbus_message_finalize (message);
}

#ifndef DBUS_DISABLE_CHECKS
static dbus_bool_t
_dbus_message_iter_check (DBusMessageRealIter *iter)
{
  if (iter == NULL)
    {
      _dbus_warn_check_failed ("dbus message iterator is NULL\n");
      return FALSE;
    }

  if (iter->iter_type == DBUS_MESSAGE_ITER_TYPE_READER)
    {
      if (iter->u.reader.byte_order != iter->message->byte_order)
        {
          _dbus_warn_check_failed ("dbus message changed byte order since iterator was created\n");
          return FALSE;
        }
      /* because we swap the message into compiler order when you init an iter */
      _dbus_assert (iter->u.reader.byte_order == DBUS_COMPILER_BYTE_ORDER);
    }
  else if (iter->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER)
    {
      if (iter->u.writer.byte_order != iter->message->byte_order)
        {
          _dbus_warn_check_failed ("dbus message changed byte order since append iterator was created\n");
          return FALSE;
        }
      /* because we swap the message into compiler order when you init an iter */
      _dbus_assert (iter->u.writer.byte_order == DBUS_COMPILER_BYTE_ORDER);
    }
  else
    {
      _dbus_warn_check_failed ("dbus message iterator looks uninitialized or corrupted\n");
      return FALSE;
    }

  if (iter->changed_stamp != iter->message->changed_stamp)
    {
      _dbus_warn_check_failed ("dbus message iterator invalid because the message has been modified (or perhaps the iterator is just uninitialized)\n");
      return FALSE;
    }

  return TRUE;
}
#endif /* DBUS_DISABLE_CHECKS */

dbus_bool_t
_dbus_message_iter_get_args_valist (DBusMessageIter *iter,
                                    DBusError       *error,
                                    int              first_arg_type,
                                    va_list          var_args)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  int spec_type, msg_type, i;
  dbus_bool_t retval;

  _dbus_assert (_dbus_message_iter_check (real));

  retval = FALSE;

  spec_type = first_arg_type;
  i = 0;

  while (spec_type != DBUS_TYPE_INVALID)
    {
      msg_type = dbus_message_iter_get_arg_type (iter);

      if (msg_type != spec_type)
	{
          dbus_set_error (error, DBUS_ERROR_INVALID_ARGS,
                          "Argument %d is specified to be of type \"%s\", but "
                          "is actually of type \"%s\"\n", i,
                          _dbus_type_to_string (spec_type),
                          _dbus_type_to_string (msg_type));

          goto out;
	}

      if (dbus_type_is_basic (spec_type))
        {
          DBusBasicValue *ptr;

          ptr = va_arg (var_args, DBusBasicValue*);

          _dbus_assert (ptr != NULL);

          _dbus_type_reader_read_basic (&real->u.reader,
                                        ptr);
        }
      else if (spec_type == DBUS_TYPE_ARRAY)
        {
          int element_type;
          int spec_element_type;
          const DBusBasicValue **ptr;
          int *n_elements_p;
          DBusTypeReader array;

          spec_element_type = va_arg (var_args, int);
          element_type = _dbus_type_reader_get_element_type (&real->u.reader);

          if (spec_element_type != element_type)
            {
              dbus_set_error (error, DBUS_ERROR_INVALID_ARGS,
                              "Argument %d is specified to be an array of \"%s\", but "
                              "is actually an array of \"%s\"\n",
                              i,
                              _dbus_type_to_string (spec_element_type),
                              _dbus_type_to_string (element_type));

              goto out;
            }

          if (dbus_type_is_fixed (spec_element_type))
            {
              ptr = va_arg (var_args, const DBusBasicValue**);
              n_elements_p = va_arg (var_args, int*);

              _dbus_assert (ptr != NULL);
              _dbus_assert (n_elements_p != NULL);

              _dbus_type_reader_recurse (&real->u.reader, &array);

              _dbus_type_reader_read_fixed_multi (&array,
                                                  ptr, n_elements_p);
            }
          else if (spec_element_type == DBUS_TYPE_STRING ||
                   spec_element_type == DBUS_TYPE_SIGNATURE ||
                   spec_element_type == DBUS_TYPE_OBJECT_PATH)
            {
              char ***str_array_p;
              int n_elements;
              char **str_array;

              str_array_p = va_arg (var_args, char***);
              n_elements_p = va_arg (var_args, int*);

              _dbus_assert (str_array_p != NULL);
              _dbus_assert (n_elements_p != NULL);

              /* Count elements in the array */
              _dbus_type_reader_recurse (&real->u.reader, &array);

              n_elements = 0;
              while (_dbus_type_reader_get_current_type (&array) != DBUS_TYPE_INVALID)
                {
                  ++n_elements;
                  _dbus_type_reader_next (&array);
                }

              str_array = dbus_new0 (char*, n_elements + 1);
              if (str_array == NULL)
                {
                  _DBUS_SET_OOM (error);
                  goto out;
                }

              /* Now go through and dup each string */
              _dbus_type_reader_recurse (&real->u.reader, &array);

              i = 0;
              while (i < n_elements)
                {
                  const char *s;
                  _dbus_type_reader_read_basic (&array,
                                                &s);
                  
                  str_array[i] = _dbus_strdup (s);
                  if (str_array[i] == NULL)
                    {
                      dbus_free_string_array (str_array);
                      _DBUS_SET_OOM (error);
                      goto out;
                    }
                  
                  ++i;
                  
                  if (!_dbus_type_reader_next (&array))
                    _dbus_assert (i == n_elements);
                }

              _dbus_assert (_dbus_type_reader_get_current_type (&array) == DBUS_TYPE_INVALID);
              _dbus_assert (i == n_elements);
              _dbus_assert (str_array[i] == NULL);

              *str_array_p = str_array;
              *n_elements_p = n_elements;
            }
#ifndef DBUS_DISABLE_CHECKS
          else
            {
              _dbus_warn ("you can't read arrays of container types (struct, variant, array) with %s for now\n",
                          _DBUS_FUNCTION_NAME);
              goto out;
            }
#endif
        }
#ifndef DBUS_DISABLE_CHECKS
      else
        {
          _dbus_warn ("you can only read arrays and basic types with %s for now\n",
                      _DBUS_FUNCTION_NAME);
          goto out;
        }
#endif

      spec_type = va_arg (var_args, int);
      if (!_dbus_type_reader_next (&real->u.reader) && spec_type != DBUS_TYPE_INVALID)
        {
          dbus_set_error (error, DBUS_ERROR_INVALID_ARGS,
                          "Message has only %d arguments, but more were expected", i);
          goto out;
        }

      i++;
    }

  retval = TRUE;

 out:

  return retval;
}

/** @} */



dbus_uint32_t
dbus_message_get_serial (DBusMessage *message)
{
  _dbus_return_val_if_fail (message != NULL, 0);

  return _dbus_header_get_serial (&message->header);
}

dbus_bool_t
dbus_message_set_reply_serial (DBusMessage   *message,
                               dbus_uint32_t  reply_serial)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (reply_serial != 0, FALSE); /* 0 is invalid */

  return _dbus_header_set_field_basic (&message->header,
                                       DBUS_HEADER_FIELD_REPLY_SERIAL,
                                       DBUS_TYPE_UINT32,
                                       &reply_serial);
}

dbus_uint32_t
dbus_message_get_reply_serial  (DBusMessage *message)
{
  dbus_uint32_t v_UINT32;

  _dbus_return_val_if_fail (message != NULL, 0);

  if (_dbus_header_get_field_basic (&message->header,
                                    DBUS_HEADER_FIELD_REPLY_SERIAL,
                                    DBUS_TYPE_UINT32,
                                    &v_UINT32))
    return v_UINT32;
  else
    return 0;
}

static void
dbus_message_finalize (DBusMessage *message)
{
  _dbus_assert (message->refcount.value == 0);

  /* This calls application callbacks! */
  _dbus_data_slot_list_free (&message->slot_list);

  _dbus_list_foreach (&message->size_counters,
                      free_size_counter, message);
  _dbus_list_clear (&message->size_counters);

  _dbus_header_free (&message->header);
  _dbus_string_free (&message->body);

  _dbus_assert (message->refcount.value == 0);
  
  dbus_free (message);
}

static DBusMessage*
dbus_message_new_empty_header (void)
{
  DBusMessage *message;
  dbus_bool_t from_cache;

  message = dbus_message_get_cached ();

  if (message != NULL)
    {
      from_cache = TRUE;
    }
  else
    {
      from_cache = FALSE;
      message = dbus_new (DBusMessage, 1);
      if (message == NULL)
        return NULL;
#ifndef DBUS_DISABLE_CHECKS
      message->generation = _dbus_current_generation;
#endif
    }
  
  message->refcount.value = 1;
  message->byte_order = DBUS_COMPILER_BYTE_ORDER;
  message->locked = FALSE;
#ifndef DBUS_DISABLE_CHECKS
  message->in_cache = FALSE;
#endif
  message->size_counters = NULL;
  message->size_counter_delta = 0;
  message->changed_stamp = 0;

  if (!from_cache)
    _dbus_data_slot_list_init (&message->slot_list);

  if (from_cache)
    {
      _dbus_header_reinit (&message->header, message->byte_order);
      _dbus_string_set_length (&message->body, 0);
    }
  else
    {
      if (!_dbus_header_init (&message->header, message->byte_order))
        {
          dbus_free (message);
          return NULL;
        }

      if (!_dbus_string_init_preallocated (&message->body, 32))
        {
          _dbus_header_free (&message->header);
          dbus_free (message);
          return NULL;
        }
    }

  return message;
}

DBusMessage*
dbus_message_new (int message_type)
{
  DBusMessage *message;

  _dbus_return_val_if_fail (message_type != DBUS_MESSAGE_TYPE_INVALID, NULL);

  message = dbus_message_new_empty_header ();
  if (message == NULL)
    return NULL;

  if (!_dbus_header_create (&message->header,
                            message_type,
                            NULL, NULL, NULL, NULL, NULL))
    {
      dbus_message_unref (message);
      return NULL;
    }

  return message;
}

DBusMessage*
dbus_message_new_method_call (const char *destination,
                              const char *path,
                              const char *interface,
                              const char *method)
{
  DBusMessage *message;

  _dbus_return_val_if_fail (path != NULL, NULL);
  _dbus_return_val_if_fail (method != NULL, NULL);
  _dbus_return_val_if_fail (destination == NULL ||
                            _dbus_check_is_valid_bus_name (destination), NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_path (path), NULL);
  _dbus_return_val_if_fail (interface == NULL ||
                            _dbus_check_is_valid_interface (interface), NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_member (method), NULL);

  message = dbus_message_new_empty_header ();
  if (message == NULL)
    return NULL;

  if (!_dbus_header_create (&message->header,
                            DBUS_MESSAGE_TYPE_METHOD_CALL,
                            destination, path, interface, method, NULL))
    {
      dbus_message_unref (message);
      return NULL;
    }

  return message;
}

DBusMessage*
dbus_message_new_method_return (DBusMessage *method_call)
{
  DBusMessage *message;
  const char *sender;

  _dbus_return_val_if_fail (method_call != NULL, NULL);

  sender = dbus_message_get_sender (method_call);

  /* sender is allowed to be null here in peer-to-peer case */

  message = dbus_message_new_empty_header ();
  if (message == NULL)
    return NULL;

  if (!_dbus_header_create (&message->header,
                            DBUS_MESSAGE_TYPE_METHOD_RETURN,
                            sender, NULL, NULL, NULL, NULL))
    {
      dbus_message_unref (message);
      return NULL;
    }

  dbus_message_set_no_reply (message, TRUE);

  if (!dbus_message_set_reply_serial (message,
                                      dbus_message_get_serial (method_call)))
    {
      dbus_message_unref (message);
      return NULL;
    }

  return message;
}

DBusMessage*
dbus_message_new_signal (const char *path,
                         const char *interface,
                         const char *name)
{
  DBusMessage *message;

  _dbus_return_val_if_fail (path != NULL, NULL);
  _dbus_return_val_if_fail (interface != NULL, NULL);
  _dbus_return_val_if_fail (name != NULL, NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_path (path), NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_interface (interface), NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_member (name), NULL);

  message = dbus_message_new_empty_header ();
  if (message == NULL)
    return NULL;

  if (!_dbus_header_create (&message->header,
                            DBUS_MESSAGE_TYPE_SIGNAL,
                            NULL, path, interface, name, NULL))
    {
      dbus_message_unref (message);
      return NULL;
    }

  dbus_message_set_no_reply (message, TRUE);

  return message;
}

DBusMessage*
dbus_message_new_error (DBusMessage *reply_to,
                        const char  *error_name,
                        const char  *error_message)
{
  DBusMessage *message;
  const char *sender;
  DBusMessageIter iter;

  _dbus_return_val_if_fail (reply_to != NULL, NULL);
  _dbus_return_val_if_fail (error_name != NULL, NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_error_name (error_name), NULL);

  sender = dbus_message_get_sender (reply_to);

  /* sender may be NULL for non-message-bus case or
   * when the message bus is dealing with an unregistered
   * connection.
   */
  message = dbus_message_new_empty_header ();
  if (message == NULL)
    return NULL;

  if (!_dbus_header_create (&message->header,
                            DBUS_MESSAGE_TYPE_ERROR,
                            sender, NULL, NULL, NULL, error_name))
    {
      dbus_message_unref (message);
      return NULL;
    }

  dbus_message_set_no_reply (message, TRUE);

  if (!dbus_message_set_reply_serial (message,
                                      dbus_message_get_serial (reply_to)))
    {
      dbus_message_unref (message);
      return NULL;
    }

  if (error_message != NULL)
    {
      dbus_message_iter_init_append (message, &iter);
      if (!dbus_message_iter_append_basic (&iter,
                                           DBUS_TYPE_STRING,
                                           &error_message))
        {
          dbus_message_unref (message);
          return NULL;
        }
    }

  return message;
}

DBusMessage*
dbus_message_new_error_printf (DBusMessage *reply_to,
			       const char  *error_name,
			       const char  *error_format,
			       ...)
{
  va_list args;
  DBusString str;
  DBusMessage *message;

  _dbus_return_val_if_fail (reply_to != NULL, NULL);
  _dbus_return_val_if_fail (error_name != NULL, NULL);
  _dbus_return_val_if_fail (_dbus_check_is_valid_error_name (error_name), NULL);

  if (!_dbus_string_init (&str))
    return NULL;

  va_start (args, error_format);

  if (_dbus_string_append_printf_valist (&str, error_format, args))
    message = dbus_message_new_error (reply_to, error_name,
				      _dbus_string_get_const_data (&str));
  else
    message = NULL;

  _dbus_string_free (&str);

  va_end (args);

  return message;
}


DBusMessage *
dbus_message_copy (const DBusMessage *message)
{
  DBusMessage *retval;

  _dbus_return_val_if_fail (message != NULL, NULL);

  retval = dbus_new0 (DBusMessage, 1);
  if (retval == NULL)
    return NULL;

  retval->refcount.value = 1;
  retval->byte_order = message->byte_order;
  retval->locked = FALSE;
#ifndef DBUS_DISABLE_CHECKS
  retval->generation = message->generation;
#endif

  if (!_dbus_header_copy (&message->header, &retval->header))
    {
      dbus_free (retval);
      return NULL;
    }

  if (!_dbus_string_init_preallocated (&retval->body,
                                       _dbus_string_get_length (&message->body)))
    {
      _dbus_header_free (&retval->header);
      dbus_free (retval);
      return NULL;
    }

  if (!_dbus_string_copy (&message->body, 0,
			  &retval->body, 0))
    goto failed_copy;

  return retval;

 failed_copy:
  _dbus_header_free (&retval->header);
  _dbus_string_free (&retval->body);
  dbus_free (retval);

  return NULL;
}


DBusMessage *
dbus_message_ref (DBusMessage *message)
{
  dbus_int32_t old_refcount;

  _dbus_return_val_if_fail (message != NULL, NULL);
  _dbus_return_val_if_fail (message->generation == _dbus_current_generation, NULL);
  _dbus_return_val_if_fail (!message->in_cache, NULL);
  
  old_refcount = _dbus_atomic_inc (&message->refcount);
  _dbus_assert (old_refcount >= 1);

  return message;
}

void
dbus_message_unref (DBusMessage *message)
{
 dbus_int32_t old_refcount;

  _dbus_return_if_fail (message != NULL);
  _dbus_return_if_fail (message->generation == _dbus_current_generation);
  _dbus_return_if_fail (!message->in_cache);

  old_refcount = _dbus_atomic_dec (&message->refcount);

  _dbus_assert (old_refcount >= 0);

  if (old_refcount == 1)
    {
      /* Calls application callbacks! */
      dbus_message_cache_or_finalize (message);
    }
}

int
dbus_message_get_type (DBusMessage *message)
{
  _dbus_return_val_if_fail (message != NULL, DBUS_MESSAGE_TYPE_INVALID);

  return _dbus_header_get_message_type (&message->header);
}

dbus_bool_t
dbus_message_append_args (DBusMessage *message,
			  int          first_arg_type,
			  ...)
{
  dbus_bool_t retval;
  va_list var_args;

  _dbus_return_val_if_fail (message != NULL, FALSE);

  va_start (var_args, first_arg_type);
  retval = dbus_message_append_args_valist (message,
					    first_arg_type,
					    var_args);
  va_end (var_args);

  return retval;
}

dbus_bool_t
dbus_message_append_args_valist (DBusMessage *message,
				 int          first_arg_type,
				 va_list      var_args)
{
  int type;
  DBusMessageIter iter;

  _dbus_return_val_if_fail (message != NULL, FALSE);

  type = first_arg_type;

  dbus_message_iter_init_append (message, &iter);

  while (type != DBUS_TYPE_INVALID)
    {
      if (dbus_type_is_basic (type))
        {
          const DBusBasicValue *value;
          value = va_arg (var_args, const DBusBasicValue*);

          if (!dbus_message_iter_append_basic (&iter,
                                               type,
                                               value))
            goto failed;
        }
      else if (type == DBUS_TYPE_ARRAY)
        {
          int element_type;
          DBusMessageIter array;
          char buf[2];

          element_type = va_arg (var_args, int);
              
          buf[0] = element_type;
          buf[1] = '\0';
          if (!dbus_message_iter_open_container (&iter,
                                                 DBUS_TYPE_ARRAY,
                                                 buf,
                                                 &array))
            goto failed;
          
          if (dbus_type_is_fixed (element_type))
            {
              const DBusBasicValue **value;
              int n_elements;

              value = va_arg (var_args, const DBusBasicValue**);
              n_elements = va_arg (var_args, int);
              
              if (!dbus_message_iter_append_fixed_array (&array,
                                                         element_type,
                                                         value,
                                                         n_elements))
                goto failed;
            }
          else if (element_type == DBUS_TYPE_STRING ||
                   element_type == DBUS_TYPE_SIGNATURE ||
                   element_type == DBUS_TYPE_OBJECT_PATH)
            {
              const char ***value_p;
              const char **value;
              int n_elements;
              int i;
              
              value_p = va_arg (var_args, const char***);
              n_elements = va_arg (var_args, int);

              value = *value_p;
              
              i = 0;
              while (i < n_elements)
                {
                  if (!dbus_message_iter_append_basic (&array,
                                                       element_type,
                                                       &value[i]))
                    goto failed;
                  ++i;
                }
            }
          else
            {
              _dbus_warn ("arrays of %s can't be appended with %s for now\n",
                          _dbus_type_to_string (element_type),
                          _DBUS_FUNCTION_NAME);
              goto failed;
            }

          if (!dbus_message_iter_close_container (&iter, &array))
            goto failed;
        }
#ifndef DBUS_DISABLE_CHECKS
      else
        {
          _dbus_warn ("type %s isn't supported yet in %s\n",
                      _dbus_type_to_string (type), _DBUS_FUNCTION_NAME);
          goto failed;
        }
#endif

      type = va_arg (var_args, int);
    }

  return TRUE;

 failed:
  return FALSE;
}

dbus_bool_t
dbus_message_get_args (DBusMessage     *message,
                       DBusError       *error,
		       int              first_arg_type,
		       ...)
{
  dbus_bool_t retval;
  va_list var_args;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_error_is_set (error, FALSE);

  va_start (var_args, first_arg_type);
  retval = dbus_message_get_args_valist (message, error, first_arg_type, var_args);
  va_end (var_args);

  return retval;
}

dbus_bool_t
dbus_message_get_args_valist (DBusMessage     *message,
                              DBusError       *error,
			      int              first_arg_type,
			      va_list          var_args)
{
  DBusMessageIter iter;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_error_is_set (error, FALSE);

  dbus_message_iter_init (message, &iter);
  return _dbus_message_iter_get_args_valist (&iter, error, first_arg_type, var_args);
}

static void
_dbus_message_iter_init_common (DBusMessage         *message,
                                DBusMessageRealIter *real,
                                int                  iter_type)
{
  _dbus_assert (sizeof (DBusMessageRealIter) <= sizeof (DBusMessageIter));

  /* Since the iterator will read or write who-knows-what from the
   * message, we need to get in the right byte order
   */
  ensure_byte_order (message);
  
  real->message = message;
  real->changed_stamp = message->changed_stamp;
  real->iter_type = iter_type;
  real->sig_refcount = 0;
}

dbus_bool_t
dbus_message_iter_init (DBusMessage     *message,
			DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  const DBusString *type_str;
  int type_pos;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (iter != NULL, FALSE);

  get_const_signature (&message->header, &type_str, &type_pos);

  _dbus_message_iter_init_common (message, real,
                                  DBUS_MESSAGE_ITER_TYPE_READER);

  _dbus_type_reader_init (&real->u.reader,
                          message->byte_order,
                          type_str, type_pos,
                          &message->body,
                          0);

  return _dbus_type_reader_get_current_type (&real->u.reader) != DBUS_TYPE_INVALID;
}

dbus_bool_t
dbus_message_iter_has_next (DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_READER, FALSE);

  return _dbus_type_reader_has_next (&real->u.reader);
}

dbus_bool_t
dbus_message_iter_next (DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_READER, FALSE);

  return _dbus_type_reader_next (&real->u.reader);
}

int
dbus_message_iter_get_arg_type (DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), DBUS_TYPE_INVALID);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_READER, FALSE);

  return _dbus_type_reader_get_current_type (&real->u.reader);
}

int
dbus_message_iter_get_element_type (DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), DBUS_TYPE_INVALID);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_READER, DBUS_TYPE_INVALID);
  _dbus_return_val_if_fail (dbus_message_iter_get_arg_type (iter) == DBUS_TYPE_ARRAY, DBUS_TYPE_INVALID);

  return _dbus_type_reader_get_element_type (&real->u.reader);
}

void
dbus_message_iter_recurse (DBusMessageIter  *iter,
                           DBusMessageIter  *sub)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  DBusMessageRealIter *real_sub = (DBusMessageRealIter *)sub;

  _dbus_return_if_fail (_dbus_message_iter_check (real));
  _dbus_return_if_fail (sub != NULL);

  *real_sub = *real;
  _dbus_type_reader_recurse (&real->u.reader, &real_sub->u.reader);
}

char *
dbus_message_iter_get_signature (DBusMessageIter *iter)
{
  const DBusString *sig;
  DBusString retstr;
  char *ret;
  int start, len;
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), NULL);

  if (!_dbus_string_init (&retstr))
    return NULL;

  _dbus_type_reader_get_signature (&real->u.reader, &sig,
				   &start, &len);
  if (!_dbus_string_append_len (&retstr,
				_dbus_string_get_const_data (sig) + start,
				len))
    return NULL;
  if (!_dbus_string_steal_data (&retstr, &ret))
    return NULL;
  _dbus_string_free (&retstr);
  return ret;
}

void
dbus_message_iter_get_basic (DBusMessageIter  *iter,
                             void             *value)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_if_fail (_dbus_message_iter_check (real));
  _dbus_return_if_fail (value != NULL);

  _dbus_type_reader_read_basic (&real->u.reader,
                                value);
}

int
dbus_message_iter_get_array_len (DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_val_if_fail (_dbus_message_iter_check (real), 0);

  return _dbus_type_reader_get_array_length (&real->u.reader);
}

void
dbus_message_iter_get_fixed_array (DBusMessageIter  *iter,
                                   void             *value,
                                   int              *n_elements)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  int subtype = _dbus_type_reader_get_current_type(&real->u.reader);

  _dbus_return_if_fail (_dbus_message_iter_check (real));
  _dbus_return_if_fail (value != NULL);
  _dbus_return_if_fail ((subtype == DBUS_TYPE_INVALID) ||
                         dbus_type_is_fixed (subtype));

  _dbus_type_reader_read_fixed_multi (&real->u.reader,
                                      value, n_elements);
}

void
dbus_message_iter_init_append (DBusMessage     *message,
			       DBusMessageIter *iter)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;

  _dbus_return_if_fail (message != NULL);
  _dbus_return_if_fail (iter != NULL);

  _dbus_message_iter_init_common (message, real,
                                  DBUS_MESSAGE_ITER_TYPE_WRITER);

  /* We create the signature string and point iterators at it "on demand"
   * when a value is actually appended. That means that init() never fails
   * due to OOM.
   */
  _dbus_type_writer_init_types_delayed (&real->u.writer,
                                        message->byte_order,
                                        &message->body,
                                        _dbus_string_get_length (&message->body));
}

static dbus_bool_t
_dbus_message_iter_open_signature (DBusMessageRealIter *real)
{
  DBusString *str;
  const DBusString *current_sig;
  int current_sig_pos;

  _dbus_assert (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER);

  if (real->u.writer.type_str != NULL)
    {
      _dbus_assert (real->sig_refcount > 0);
      real->sig_refcount += 1;
      return TRUE;
    }

  str = dbus_new (DBusString, 1);
  if (str == NULL)
    return FALSE;

  if (!_dbus_header_get_field_raw (&real->message->header,
                                   DBUS_HEADER_FIELD_SIGNATURE,
                                   &current_sig, &current_sig_pos))
    current_sig = NULL;

  if (current_sig)
    {
      int current_len;

      current_len = _dbus_string_get_byte (current_sig, current_sig_pos);
      current_sig_pos += 1; /* move on to sig data */

      if (!_dbus_string_init_preallocated (str, current_len + 4))
        {
          dbus_free (str);
          return FALSE;
        }

      if (!_dbus_string_copy_len (current_sig, current_sig_pos, current_len,
                                  str, 0))
        {
          _dbus_string_free (str);
          dbus_free (str);
          return FALSE;
        }
    }
  else
    {
      if (!_dbus_string_init_preallocated (str, 4))
        {
          dbus_free (str);
          return FALSE;
        }
    }

  real->sig_refcount = 1;

  _dbus_type_writer_add_types (&real->u.writer,
                               str, _dbus_string_get_length (str));
  return TRUE;
}

static dbus_bool_t
_dbus_message_iter_close_signature (DBusMessageRealIter *real)
{
  DBusString *str;
  const char *v_STRING;
  dbus_bool_t retval;

  _dbus_assert (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER);
  _dbus_assert (real->u.writer.type_str != NULL);
  _dbus_assert (real->sig_refcount > 0);

  real->sig_refcount -= 1;

  if (real->sig_refcount > 0)
    return TRUE;
  _dbus_assert (real->sig_refcount == 0);

  retval = TRUE;

  str = real->u.writer.type_str;

  v_STRING = _dbus_string_get_const_data (str);
  if (!_dbus_header_set_field_basic (&real->message->header,
                                     DBUS_HEADER_FIELD_SIGNATURE,
                                     DBUS_TYPE_SIGNATURE,
                                     &v_STRING))
    retval = FALSE;

  _dbus_type_writer_remove_types (&real->u.writer);
  _dbus_string_free (str);
  dbus_free (str);

  return retval;
}

#ifndef DBUS_DISABLE_CHECKS
static dbus_bool_t
_dbus_message_iter_append_check (DBusMessageRealIter *iter)
{
  if (!_dbus_message_iter_check (iter))
    return FALSE;

  if (iter->message->locked)
    {
      _dbus_warn_check_failed ("dbus append iterator can't be used: message is locked (has already been sent)\n");
      return FALSE;
    }

  return TRUE;
}
#endif /* DBUS_DISABLE_CHECKS */

dbus_bool_t
dbus_message_iter_append_basic (DBusMessageIter *iter,
                                int              type,
                                const void      *value)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  dbus_bool_t ret;

  _dbus_return_val_if_fail (_dbus_message_iter_append_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER, FALSE);
  _dbus_return_val_if_fail (dbus_type_is_basic (type), FALSE);
  _dbus_return_val_if_fail (value != NULL, FALSE);

  if (!_dbus_message_iter_open_signature (real))
    return FALSE;

  ret = _dbus_type_writer_write_basic (&real->u.writer, type, value);

  if (!_dbus_message_iter_close_signature (real))
    ret = FALSE;

  return ret;
}

dbus_bool_t
dbus_message_iter_append_fixed_array (DBusMessageIter *iter,
                                      int              element_type,
                                      const void      *value,
                                      int              n_elements)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  dbus_bool_t ret;

  _dbus_return_val_if_fail (_dbus_message_iter_append_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER, FALSE);
  _dbus_return_val_if_fail (dbus_type_is_fixed (element_type), FALSE);
  _dbus_return_val_if_fail (real->u.writer.container_type == DBUS_TYPE_ARRAY, FALSE);
  _dbus_return_val_if_fail (value != NULL, FALSE);
  _dbus_return_val_if_fail (n_elements >= 0, FALSE);
  _dbus_return_val_if_fail (n_elements <=
                            DBUS_MAXIMUM_ARRAY_LENGTH / _dbus_type_get_alignment (element_type),
                            FALSE);

  ret = _dbus_type_writer_write_fixed_multi (&real->u.writer, element_type, value, n_elements);

  return ret;
}

dbus_bool_t
dbus_message_iter_open_container (DBusMessageIter *iter,
                                  int              type,
                                  const char      *contained_signature,
                                  DBusMessageIter *sub)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  DBusMessageRealIter *real_sub = (DBusMessageRealIter *)sub;
  DBusString contained_str;

  _dbus_return_val_if_fail (_dbus_message_iter_append_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER, FALSE);
  _dbus_return_val_if_fail (dbus_type_is_container (type), FALSE);
  _dbus_return_val_if_fail (sub != NULL, FALSE);
  _dbus_return_val_if_fail ((type == DBUS_TYPE_STRUCT &&
                             contained_signature == NULL) ||
                            (type == DBUS_TYPE_DICT_ENTRY &&
                             contained_signature == NULL) ||
                            contained_signature != NULL, FALSE);
  
#if 0
  /* FIXME this would fail if the contained_signature is a dict entry,
   * since dict entries are invalid signatures standalone (they must be in
   * an array)
   */
  _dbus_return_val_if_fail (contained_signature == NULL ||
                            _dbus_check_is_valid_signature (contained_signature));
#endif

  if (!_dbus_message_iter_open_signature (real))
    return FALSE;

  *real_sub = *real;

  if (contained_signature != NULL)
    {
      _dbus_string_init_const (&contained_str, contained_signature);

      return _dbus_type_writer_recurse (&real->u.writer,
                                        type,
                                        &contained_str, 0,
                                        &real_sub->u.writer);
    }
  else
    {
      return _dbus_type_writer_recurse (&real->u.writer,
                                        type,
                                        NULL, 0,
                                        &real_sub->u.writer);
    } 
}


dbus_bool_t
dbus_message_iter_close_container (DBusMessageIter *iter,
                                   DBusMessageIter *sub)
{
  DBusMessageRealIter *real = (DBusMessageRealIter *)iter;
  DBusMessageRealIter *real_sub = (DBusMessageRealIter *)sub;
  dbus_bool_t ret;

  _dbus_return_val_if_fail (_dbus_message_iter_append_check (real), FALSE);
  _dbus_return_val_if_fail (real->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER, FALSE);
  _dbus_return_val_if_fail (_dbus_message_iter_append_check (real_sub), FALSE);
  _dbus_return_val_if_fail (real_sub->iter_type == DBUS_MESSAGE_ITER_TYPE_WRITER, FALSE);

  ret = _dbus_type_writer_unrecurse (&real->u.writer,
                                     &real_sub->u.writer);

  if (!_dbus_message_iter_close_signature (real))
    ret = FALSE;

  return ret;
}

void
dbus_message_set_no_reply (DBusMessage *message,
                           dbus_bool_t  no_reply)
{
  _dbus_return_if_fail (message != NULL);
  _dbus_return_if_fail (!message->locked);

  _dbus_header_toggle_flag (&message->header,
                            DBUS_HEADER_FLAG_NO_REPLY_EXPECTED,
                            no_reply);
}

dbus_bool_t
dbus_message_get_no_reply (DBusMessage *message)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);

  return _dbus_header_get_flag (&message->header,
                                DBUS_HEADER_FLAG_NO_REPLY_EXPECTED);
}

void
dbus_message_set_auto_start (DBusMessage *message,
                             dbus_bool_t  auto_start)
{
  _dbus_return_if_fail (message != NULL);
  _dbus_return_if_fail (!message->locked);

  _dbus_header_toggle_flag (&message->header,
                            DBUS_HEADER_FLAG_NO_AUTO_START,
                            !auto_start);
}

dbus_bool_t
dbus_message_get_auto_start (DBusMessage *message)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);

  return !_dbus_header_get_flag (&message->header,
                                 DBUS_HEADER_FLAG_NO_AUTO_START);
}


dbus_bool_t
dbus_message_set_path (DBusMessage   *message,
                       const char    *object_path)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (object_path == NULL ||
                            _dbus_check_is_valid_path (object_path),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_PATH,
                                     DBUS_TYPE_OBJECT_PATH,
                                     object_path);
}

const char*
dbus_message_get_path (DBusMessage   *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_PATH,
                                DBUS_TYPE_OBJECT_PATH,
                                &v);
  return v;
}

dbus_bool_t
dbus_message_has_path (DBusMessage   *message,
                       const char    *path)
{
  const char *msg_path;
  msg_path = dbus_message_get_path (message);
  
  if (msg_path == NULL)
    {
      if (path == NULL)
        return TRUE;
      else
        return FALSE;
    }

  if (path == NULL)
    return FALSE;
   
  if (strcmp (msg_path, path) == 0)
    return TRUE;

  return FALSE;
}

dbus_bool_t
dbus_message_get_path_decomposed (DBusMessage   *message,
                                  char        ***path)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (path != NULL, FALSE);

  *path = NULL;

  v = dbus_message_get_path (message);
  if (v != NULL)
    {
      if (!_dbus_decompose_path (v, strlen (v),
                                 path, NULL))
        return FALSE;
    }
  return TRUE;
}

dbus_bool_t
dbus_message_set_interface (DBusMessage  *message,
                            const char   *interface)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (interface == NULL ||
                            _dbus_check_is_valid_interface (interface),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_INTERFACE,
                                     DBUS_TYPE_STRING,
                                     interface);
}

const char*
dbus_message_get_interface (DBusMessage *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_INTERFACE,
                                DBUS_TYPE_STRING,
                                &v);
  return v;
}

dbus_bool_t
dbus_message_has_interface (DBusMessage   *message,
                            const char    *interface)
{
  const char *msg_interface;
  msg_interface = dbus_message_get_interface (message);
   
  if (msg_interface == NULL)
    {
      if (interface == NULL)
        return TRUE;
      else
        return FALSE;
    }

  if (interface == NULL)
    return FALSE;
     
  if (strcmp (msg_interface, interface) == 0)
    return TRUE;

  return FALSE;

}

dbus_bool_t
dbus_message_set_member (DBusMessage  *message,
                         const char   *member)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (member == NULL ||
                            _dbus_check_is_valid_member (member),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_MEMBER,
                                     DBUS_TYPE_STRING,
                                     member);
}

const char*
dbus_message_get_member (DBusMessage *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_MEMBER,
                                DBUS_TYPE_STRING,
                                &v);
  return v;
}

dbus_bool_t
dbus_message_has_member (DBusMessage   *message,
                         const char    *member)
{
  const char *msg_member;
  msg_member = dbus_message_get_member (message);
 
  if (msg_member == NULL)
    {
      if (member == NULL)
        return TRUE;
      else
        return FALSE;
    }

  if (member == NULL)
    return FALSE;
    
  if (strcmp (msg_member, member) == 0)
    return TRUE;

  return FALSE;

}

dbus_bool_t
dbus_message_set_error_name (DBusMessage  *message,
                             const char   *error_name)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (error_name == NULL ||
                            _dbus_check_is_valid_error_name (error_name),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_ERROR_NAME,
                                     DBUS_TYPE_STRING,
                                     error_name);
}

const char*
dbus_message_get_error_name (DBusMessage *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_ERROR_NAME,
                                DBUS_TYPE_STRING,
                                &v);
  return v;
}

dbus_bool_t
dbus_message_set_destination (DBusMessage  *message,
                              const char   *destination)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (destination == NULL ||
                            _dbus_check_is_valid_bus_name (destination),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_DESTINATION,
                                     DBUS_TYPE_STRING,
                                     destination);
}

const char*
dbus_message_get_destination (DBusMessage *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_DESTINATION,
                                DBUS_TYPE_STRING,
                                &v);
  return v;
}

dbus_bool_t
dbus_message_set_sender (DBusMessage  *message,
                         const char   *sender)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (!message->locked, FALSE);
  _dbus_return_val_if_fail (sender == NULL ||
                            _dbus_check_is_valid_bus_name (sender),
                            FALSE);

  return set_or_delete_string_field (message,
                                     DBUS_HEADER_FIELD_SENDER,
                                     DBUS_TYPE_STRING,
                                     sender);
}

const char*
dbus_message_get_sender (DBusMessage *message)
{
  const char *v;

  _dbus_return_val_if_fail (message != NULL, NULL);

  v = NULL; /* in case field doesn't exist */
  _dbus_header_get_field_basic (&message->header,
                                DBUS_HEADER_FIELD_SENDER,
                                DBUS_TYPE_STRING,
                                &v);
  return v;
}

const char*
dbus_message_get_signature (DBusMessage *message)
{
  const DBusString *type_str;
  int type_pos;

  _dbus_return_val_if_fail (message != NULL, NULL);

  get_const_signature (&message->header, &type_str, &type_pos);

  return _dbus_string_get_const_data_len (type_str, type_pos, 0);
}

static dbus_bool_t
_dbus_message_has_type_interface_member (DBusMessage *message,
                                         int          type,
                                         const char  *interface,
                                         const char  *member)
{
  const char *n;

  _dbus_assert (message != NULL);
  _dbus_assert (interface != NULL);
  _dbus_assert (member != NULL);

  if (dbus_message_get_type (message) != type)
    return FALSE;

  /* Optimize by checking the short member name first
   * instead of the longer interface name
   */

  n = dbus_message_get_member (message);

  if (n && strcmp (n, member) == 0)
    {
      n = dbus_message_get_interface (message);

      if (n == NULL || strcmp (n, interface) == 0)
        return TRUE;
    }

  return FALSE;
}

dbus_bool_t
dbus_message_is_method_call (DBusMessage *message,
                             const char  *interface,
                             const char  *method)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (interface != NULL, FALSE);
  _dbus_return_val_if_fail (method != NULL, FALSE);
  /* don't check that interface/method are valid since it would be
   * expensive, and not catch many common errors
   */

  return _dbus_message_has_type_interface_member (message,
                                                  DBUS_MESSAGE_TYPE_METHOD_CALL,
                                                  interface, method);
}

dbus_bool_t
dbus_message_is_signal (DBusMessage *message,
                        const char  *interface,
                        const char  *signal_name)
{
  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (interface != NULL, FALSE);
  _dbus_return_val_if_fail (signal_name != NULL, FALSE);
  /* don't check that interface/name are valid since it would be
   * expensive, and not catch many common errors
   */

  return _dbus_message_has_type_interface_member (message,
                                                  DBUS_MESSAGE_TYPE_SIGNAL,
                                                  interface, signal_name);
}

dbus_bool_t
dbus_message_is_error (DBusMessage *message,
                       const char  *error_name)
{
  const char *n;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (error_name != NULL, FALSE);
  /* don't check that error_name is valid since it would be expensive,
   * and not catch many common errors
   */

  if (dbus_message_get_type (message) != DBUS_MESSAGE_TYPE_ERROR)
    return FALSE;

  n = dbus_message_get_error_name (message);

  if (n && strcmp (n, error_name) == 0)
    return TRUE;
  else
    return FALSE;
}

dbus_bool_t
dbus_message_has_destination (DBusMessage  *message,
                              const char   *name)
{
  const char *s;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (name != NULL, FALSE);
  /* don't check that name is valid since it would be expensive, and
   * not catch many common errors
   */

  s = dbus_message_get_destination (message);

  if (s && strcmp (s, name) == 0)
    return TRUE;
  else
    return FALSE;
}

dbus_bool_t
dbus_message_has_sender (DBusMessage  *message,
                         const char   *name)
{
  const char *s;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (name != NULL, FALSE);
  /* don't check that name is valid since it would be expensive, and
   * not catch many common errors
   */

  s = dbus_message_get_sender (message);

  if (s && strcmp (s, name) == 0)
    return TRUE;
  else
    return FALSE;
}

dbus_bool_t
dbus_message_has_signature (DBusMessage   *message,
                            const char    *signature)
{
  const char *s;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (signature != NULL, FALSE);
  /* don't check that signature is valid since it would be expensive,
   * and not catch many common errors
   */

  s = dbus_message_get_signature (message);

  if (s && strcmp (s, signature) == 0)
    return TRUE;
  else
    return FALSE;
}

dbus_bool_t
dbus_set_error_from_message (DBusError   *error,
                             DBusMessage *message)
{
  const char *str;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_error_is_set (error, FALSE);

  if (dbus_message_get_type (message) != DBUS_MESSAGE_TYPE_ERROR)
    return FALSE;

  str = NULL;
  dbus_message_get_args (message, NULL,
                         DBUS_TYPE_STRING, &str,
                         DBUS_TYPE_INVALID);

  dbus_set_error (error, dbus_message_get_error_name (message),
                  str ? "%s" : NULL, str);

  return TRUE;
}

/** @} */


#define INITIAL_LOADER_DATA_LEN 32

DBusMessageLoader*
_dbus_message_loader_new (void)
{
  DBusMessageLoader *loader;

  loader = dbus_new0 (DBusMessageLoader, 1);
  if (loader == NULL)
    return NULL;
  
  loader->refcount = 1;

  loader->corrupted = FALSE;
  loader->corruption_reason = DBUS_VALID;

  /* this can be configured by the app, but defaults to the protocol max */
  loader->max_message_size = DBUS_MAXIMUM_MESSAGE_LENGTH;

  if (!_dbus_string_init (&loader->data))
    {
      dbus_free (loader);
      return NULL;
    }

  /* preallocate the buffer for speed, ignore failure */
  _dbus_string_set_length (&loader->data, INITIAL_LOADER_DATA_LEN);
  _dbus_string_set_length (&loader->data, 0);

  return loader;
}

DBusMessageLoader *
_dbus_message_loader_ref (DBusMessageLoader *loader)
{
  loader->refcount += 1;

  return loader;
}

void
_dbus_message_loader_unref (DBusMessageLoader *loader)
{
  loader->refcount -= 1;
  if (loader->refcount == 0)
    {
      _dbus_list_foreach (&loader->messages,
                          (DBusForeachFunction) dbus_message_unref,
                          NULL);
      _dbus_list_clear (&loader->messages);
      _dbus_string_free (&loader->data);
      dbus_free (loader);
    }
}

void
_dbus_message_loader_get_buffer (DBusMessageLoader  *loader,
                                 DBusString        **buffer)
{
  _dbus_assert (!loader->buffer_outstanding);

  *buffer = &loader->data;

  loader->buffer_outstanding = TRUE;
}

void
_dbus_message_loader_return_buffer (DBusMessageLoader  *loader,
                                    DBusString         *buffer,
                                    int                 bytes_read)
{
  _dbus_assert (loader->buffer_outstanding);
  _dbus_assert (buffer == &loader->data);

  loader->buffer_outstanding = FALSE;
}

static dbus_bool_t
load_message (DBusMessageLoader *loader,
              DBusMessage       *message,
              int                byte_order,
              int                fields_array_len,
              int                header_len,
              int                body_len)
{
  dbus_bool_t oom;
  DBusValidity validity;
  const DBusString *type_str;
  int type_pos;
  DBusValidationMode mode;

  mode = DBUS_VALIDATION_MODE_DATA_IS_UNTRUSTED;
  
  oom = FALSE;

#if 0
  _dbus_verbose_bytes_of_string (&loader->data, 0, header_len /* + body_len */);
#endif

  /* 1. VALIDATE AND COPY OVER HEADER */
  _dbus_assert (_dbus_string_get_length (&message->header.data) == 0);
  _dbus_assert ((header_len + body_len) <= _dbus_string_get_length (&loader->data));

  if (!_dbus_header_load (&message->header,
                          mode,
                          &validity,
                          byte_order,
                          fields_array_len,
                          header_len,
                          body_len,
                          &loader->data, 0,
                          _dbus_string_get_length (&loader->data)))
    {
      _dbus_verbose ("Failed to load header for new message code %d\n", validity);

      /* assert here so we can catch any code that still uses DBUS_VALID to indicate
         oom errors.  They should use DBUS_VALIDITY_UNKNOWN_OOM_ERROR instead */
      _dbus_assert (validity != DBUS_VALID);

      if (validity == DBUS_VALIDITY_UNKNOWN_OOM_ERROR)
        oom = TRUE;
      else
        {
          loader->corrupted = TRUE;
          loader->corruption_reason = validity;
        }
      goto failed;
    }

  _dbus_assert (validity == DBUS_VALID);

  message->byte_order = byte_order;

  /* 2. VALIDATE BODY */
  if (mode != DBUS_VALIDATION_MODE_WE_TRUST_THIS_DATA_ABSOLUTELY)
    {
      get_const_signature (&message->header, &type_str, &type_pos);
      
      /* Because the bytes_remaining arg is NULL, this validates that the
       * body is the right length
       */
      validity = _dbus_validate_body_with_reason (type_str,
                                                  type_pos,
                                                  byte_order,
                                                  NULL,
                                                  &loader->data,
                                                  header_len,
                                                  body_len);
      if (validity != DBUS_VALID)
        {
          _dbus_verbose ("Failed to validate message body code %d\n", validity);

          loader->corrupted = TRUE;
          loader->corruption_reason = validity;
          
          goto failed;
        }
    }

  /* 3. COPY OVER BODY AND QUEUE MESSAGE */

  if (!_dbus_list_append (&loader->messages, message))
    {
      _dbus_verbose ("Failed to append new message to loader queue\n");
      oom = TRUE;
      goto failed;
    }

  _dbus_assert (_dbus_string_get_length (&message->body) == 0);
  _dbus_assert (_dbus_string_get_length (&loader->data) >=
                (header_len + body_len));

  if (!_dbus_string_copy_len (&loader->data, header_len, body_len, &message->body, 0))
    {
      _dbus_verbose ("Failed to move body into new message\n");
      oom = TRUE;
      goto failed;
    }

  _dbus_string_delete (&loader->data, 0, header_len + body_len);

  _dbus_assert (_dbus_string_get_length (&message->header.data) == header_len);
  _dbus_assert (_dbus_string_get_length (&message->body) == body_len);

  _dbus_verbose ("Loaded message %p\n", message);

  _dbus_assert (!oom);
  _dbus_assert (!loader->corrupted);
  _dbus_assert (loader->messages != NULL);
  _dbus_assert (_dbus_list_find_last (&loader->messages, message) != NULL);

  return TRUE;

 failed:

  /* Clean up */

  /* does nothing if the message isn't in the list */
  _dbus_list_remove_last (&loader->messages, message);
  
  if (oom)
    _dbus_assert (!loader->corrupted);
  else
    _dbus_assert (loader->corrupted);

  _dbus_verbose_bytes_of_string (&loader->data, 0, _dbus_string_get_length (&loader->data));

  return FALSE;
}

dbus_bool_t
_dbus_message_loader_queue_messages (DBusMessageLoader *loader)
{
  while (!loader->corrupted &&
         _dbus_string_get_length (&loader->data) >= DBUS_MINIMUM_HEADER_SIZE)
    {
      DBusValidity validity;
      int byte_order, fields_array_len, header_len, body_len;

      if (_dbus_header_have_message_untrusted (loader->max_message_size,
                                               &validity,
                                               &byte_order,
                                               &fields_array_len,
                                               &header_len,
                                               &body_len,
                                               &loader->data, 0,
                                               _dbus_string_get_length (&loader->data)))
        {
          DBusMessage *message;

          _dbus_assert (validity == DBUS_VALID);

          message = dbus_message_new_empty_header ();
          if (message == NULL)
            return FALSE;

          if (!load_message (loader, message,
                             byte_order, fields_array_len,
                             header_len, body_len))
            {
              dbus_message_unref (message);
              /* load_message() returns false if corrupted or OOM; if
               * corrupted then return TRUE for not OOM
               */
              return loader->corrupted;
            }

          _dbus_assert (loader->messages != NULL);
          _dbus_assert (_dbus_list_find_last (&loader->messages, message) != NULL);
	}
      else
        {
          _dbus_verbose ("Initial peek at header says we don't have a whole message yet, or data broken with invalid code %d\n",
                         validity);
          if (validity != DBUS_VALID)
            {
              loader->corrupted = TRUE;
              loader->corruption_reason = validity;
            }
          return TRUE;
        }
    }

  return TRUE;
}

DBusMessage*
_dbus_message_loader_peek_message (DBusMessageLoader *loader)
{
  if (loader->messages)
    return loader->messages->data;
  else
    return NULL;
}

DBusMessage*
_dbus_message_loader_pop_message (DBusMessageLoader *loader)
{
  return _dbus_list_pop_first (&loader->messages);
}

DBusList*
_dbus_message_loader_pop_message_link (DBusMessageLoader *loader)
{
  return _dbus_list_pop_first_link (&loader->messages);
}

void
_dbus_message_loader_putback_message_link (DBusMessageLoader  *loader,
                                           DBusList           *link)
{
  _dbus_list_prepend_link (&loader->messages, link);
}

dbus_bool_t
_dbus_message_loader_get_is_corrupted (DBusMessageLoader *loader)
{
  _dbus_assert ((loader->corrupted && loader->corruption_reason != DBUS_VALID) ||
                (!loader->corrupted && loader->corruption_reason == DBUS_VALID));
  return loader->corrupted;
}

void
_dbus_message_loader_set_max_message_size (DBusMessageLoader  *loader,
                                           long                size)
{
  if (size > DBUS_MAXIMUM_MESSAGE_LENGTH)
    {
      _dbus_verbose ("clamping requested max message size %ld to %d\n",
                     size, DBUS_MAXIMUM_MESSAGE_LENGTH);
      size = DBUS_MAXIMUM_MESSAGE_LENGTH;
    }
  loader->max_message_size = size;
}

long
_dbus_message_loader_get_max_message_size (DBusMessageLoader  *loader)
{
  return loader->max_message_size;
}

static DBusDataSlotAllocator slot_allocator;
_DBUS_DEFINE_GLOBAL_LOCK (message_slots);

dbus_bool_t
dbus_message_allocate_data_slot (dbus_int32_t *slot_p)
{
  return _dbus_data_slot_allocator_alloc (&slot_allocator,
                                          &_DBUS_LOCK_NAME (message_slots),
                                          slot_p);
}

void
dbus_message_free_data_slot (dbus_int32_t *slot_p)
{
  _dbus_return_if_fail (*slot_p >= 0);

  _dbus_data_slot_allocator_free (&slot_allocator, slot_p);
}

dbus_bool_t
dbus_message_set_data (DBusMessage     *message,
                       dbus_int32_t     slot,
                       void            *data,
                       DBusFreeFunction free_data_func)
{
  DBusFreeFunction old_free_func;
  void *old_data;
  dbus_bool_t retval;

  _dbus_return_val_if_fail (message != NULL, FALSE);
  _dbus_return_val_if_fail (slot >= 0, FALSE);

  retval = _dbus_data_slot_list_set (&slot_allocator,
                                     &message->slot_list,
                                     slot, data, free_data_func,
                                     &old_free_func, &old_data);

  if (retval)
    {
      /* Do the actual free outside the message lock */
      if (old_free_func)
        (* old_free_func) (old_data);
    }

  return retval;
}

void*
dbus_message_get_data (DBusMessage   *message,
                       dbus_int32_t   slot)
{
  void *res;

  _dbus_return_val_if_fail (message != NULL, NULL);

  res = _dbus_data_slot_list_get (&slot_allocator,
                                  &message->slot_list,
                                  slot);

  return res;
}

int
dbus_message_type_from_string (const char *type_str)
{
  if (strcmp (type_str, "method_call") == 0)
    return DBUS_MESSAGE_TYPE_METHOD_CALL;
  if (strcmp (type_str, "method_return") == 0)
    return DBUS_MESSAGE_TYPE_METHOD_RETURN;
  else if (strcmp (type_str, "signal") == 0)
    return DBUS_MESSAGE_TYPE_SIGNAL;
  else if (strcmp (type_str, "error") == 0)
    return DBUS_MESSAGE_TYPE_ERROR;
  else
    return DBUS_MESSAGE_TYPE_INVALID;
}

const char *
dbus_message_type_to_string (int type)
{
  switch (type)
    {
    case DBUS_MESSAGE_TYPE_METHOD_CALL:
      return "method_call";
    case DBUS_MESSAGE_TYPE_METHOD_RETURN:
      return "method_return";
    case DBUS_MESSAGE_TYPE_SIGNAL:
      return "signal";
    case DBUS_MESSAGE_TYPE_ERROR:
      return "error";
    default:
      return "invalid";
    }
}

/** @} */

/* tests in dbus-message-util.c */
