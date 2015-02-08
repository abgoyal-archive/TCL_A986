
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_SHA_H
#define DBUS_SHA_H

#include <dbus/dbus-macros.h>
#include <dbus/dbus-errors.h>
#include <dbus/dbus-string.h>

DBUS_BEGIN_DECLS

typedef struct DBusSHAContext DBusSHAContext;

struct DBusSHAContext
{
  dbus_uint32_t  digest[5];         /**< Message digest */
  dbus_uint32_t  count_lo;          /**< 64-bit bit count */
  dbus_uint32_t  count_hi;          /**< No clue */
  dbus_uint32_t  data[16];          /**< SHA data buffer */
};

void        _dbus_sha_init    (DBusSHAContext   *context);
void        _dbus_sha_update  (DBusSHAContext   *context,
                               const DBusString *data);
dbus_bool_t _dbus_sha_final   (DBusSHAContext   *context,
                               DBusString       *results);
dbus_bool_t _dbus_sha_compute (const DBusString *data,
                               DBusString       *ascii_output);

DBUS_END_DECLS

#endif /* DBUS_SHA_H */
