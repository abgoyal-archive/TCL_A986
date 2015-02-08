
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_ERROR_H
#define DBUS_ERROR_H

#include <dbus/dbus-macros.h>
#include <dbus/dbus-types.h>

DBUS_BEGIN_DECLS


/** Mostly-opaque type representing an error that occurred */
typedef struct DBusError DBusError;

struct DBusError
{
  const char *name;    /**< public error name field */
  const char *message; /**< public error message field */

  unsigned int dummy1 : 1; /**< placeholder */
  unsigned int dummy2 : 1; /**< placeholder */
  unsigned int dummy3 : 1; /**< placeholder */
  unsigned int dummy4 : 1; /**< placeholder */
  unsigned int dummy5 : 1; /**< placeholder */

  void *padding1; /**< placeholder */
};

void        dbus_error_init      (DBusError       *error);
void        dbus_error_free      (DBusError       *error);
void        dbus_set_error       (DBusError       *error,
                                  const char      *name,
                                  const char      *message,
                                  ...);
void        dbus_set_error_const (DBusError       *error,
                                  const char      *name,
                                  const char      *message);
void        dbus_move_error      (DBusError       *src,
                                  DBusError       *dest);
dbus_bool_t dbus_error_has_name  (const DBusError *error,
                                  const char      *name);
dbus_bool_t dbus_error_is_set    (const DBusError *error);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_ERROR_H */
