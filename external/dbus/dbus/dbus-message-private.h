
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_MESSAGE_PRIVATE_H
#define DBUS_MESSAGE_PRIVATE_H

#include <dbus/dbus-message.h>
#include <dbus/dbus-message-internal.h>
#include <dbus/dbus-string.h>
#include <dbus/dbus-dataslot.h>
#include <dbus/dbus-marshal-header.h>

DBUS_BEGIN_DECLS



struct DBusMessageLoader
{
  int refcount;        /**< Reference count. */

  DBusString data;     /**< Buffered data */

  DBusList *messages;  /**< Complete messages. */

  long max_message_size; /**< Maximum size of a message */

  unsigned int buffer_outstanding : 1; /**< Someone is using the buffer to read */

  unsigned int corrupted : 1; /**< We got broken data, and are no longer working */

  DBusValidity corruption_reason; /**< why we were corrupted */
};


/** How many bits are in the changed_stamp used to validate iterators */
#define CHANGED_STAMP_BITS 21

struct DBusMessage
{
  DBusAtomic refcount; /**< Reference count */

  DBusHeader header; /**< Header network data and associated cache */

  DBusString body;   /**< Body network data. */

  char byte_order; /**< Message byte order. */

  unsigned int locked : 1; /**< Message being sent, no modifications allowed. */

#ifndef DBUS_DISABLE_CHECKS
  unsigned int in_cache : 1; /**< Has been "freed" since it's in the cache (this is a debug feature) */
#endif
  
  DBusList *size_counters;   /**< 0-N DBusCounter used to track message size. */
  long size_counter_delta;   /**< Size we incremented the size counters by.   */

  dbus_uint32_t changed_stamp : CHANGED_STAMP_BITS; /**< Incremented when iterators are invalidated. */

  DBusDataSlotList slot_list;   /**< Data stored by allocated integer ID */

#ifndef DBUS_DISABLE_CHECKS
  int generation; /**< _dbus_current_generation when message was created */
#endif
};

dbus_bool_t _dbus_message_iter_get_args_valist (DBusMessageIter *iter,
                                                DBusError       *error,
                                                int              first_arg_type,
                                                va_list          var_args);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_MESSAGE_H */
