
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_MEMORY_H
#define DBUS_MEMORY_H

#include <dbus/dbus-macros.h>
#include <stddef.h>

DBUS_BEGIN_DECLS


void* dbus_malloc        (size_t bytes);
void* dbus_malloc0       (size_t bytes);
void* dbus_realloc       (void  *memory,
                          size_t bytes);
void  dbus_free          (void  *memory);

#define dbus_new(type, count)  ((type*)dbus_malloc (sizeof (type) * (count)));
#define dbus_new0(type, count) ((type*)dbus_malloc0 (sizeof (type) * (count)));

void dbus_free_string_array (char **str_array);

typedef void (* DBusFreeFunction) (void *memory);

void dbus_shutdown (void);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_MEMORY_H */
