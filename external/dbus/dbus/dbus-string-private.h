
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_STRING_PRIVATE_H
#define DBUS_STRING_PRIVATE_H

#include <config.h>

#include <dbus/dbus-memory.h>
#include <dbus/dbus-types.h>

#ifndef DBUS_CAN_USE_DBUS_STRING_PRIVATE
#error "Don't go including dbus-string-private.h for no good reason"
#endif

DBUS_BEGIN_DECLS

typedef struct
{
  unsigned char *str;            /**< String data, plus nul termination */
  int            len;            /**< Length without nul */
  int            allocated;      /**< Allocated size of data */
  int            max_length;     /**< Max length of this string, without nul byte */
  unsigned int   constant : 1;   /**< String data is not owned by DBusString */
  unsigned int   locked : 1;     /**< DBusString has been locked and can't be changed */
  unsigned int   invalid : 1;    /**< DBusString is invalid (e.g. already freed) */
  unsigned int   align_offset : 3; /**< str - align_offset is the actual malloc block */
} DBusRealString;



#define _DBUS_STRING_MAX_MAX_LENGTH (_DBUS_INT32_MAX - _DBUS_STRING_ALLOCATION_PADDING)

#define DBUS_GENERIC_STRING_PREAMBLE(real) _dbus_assert ((real) != NULL); _dbus_assert (!(real)->invalid); _dbus_assert ((real)->len >= 0); _dbus_assert ((real)->allocated >= 0); _dbus_assert ((real)->max_length >= 0); _dbus_assert ((real)->len <= ((real)->allocated - _DBUS_STRING_ALLOCATION_PADDING)); _dbus_assert ((real)->len <= (real)->max_length)

#define DBUS_STRING_PREAMBLE(str) DBusRealString *real = (DBusRealString*) str; \
  DBUS_GENERIC_STRING_PREAMBLE (real);                                          \
  _dbus_assert (!(real)->constant);                                             \
  _dbus_assert (!(real)->locked)

#define DBUS_LOCKED_STRING_PREAMBLE(str) DBusRealString *real = (DBusRealString*) str; \
  DBUS_GENERIC_STRING_PREAMBLE (real);                                                 \
  _dbus_assert (!(real)->constant)

#define DBUS_CONST_STRING_PREAMBLE(str) const DBusRealString *real = (DBusRealString*) str; \
  DBUS_GENERIC_STRING_PREAMBLE (real)

#define DBUS_IS_ASCII_BLANK(c) ((c) == ' ' || (c) == '\t')

#define DBUS_IS_ASCII_WHITE(c) ((c) == ' ' || (c) == '\t' || (c) == '\n' || (c) == '\r')

/** @} */

DBUS_END_DECLS

#endif /* DBUS_STRING_PRIVATE_H */
