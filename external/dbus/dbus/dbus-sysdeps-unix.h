
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_SYSDEPS_UNIX_H
#define DBUS_SYSDEPS_UNIX_H

#include <config.h>
#include <dbus/dbus-sysdeps.h>

#ifdef DBUS_WIN
#error "Don't include this on Windows"
#endif

DBUS_BEGIN_DECLS


dbus_bool_t 
_dbus_close     (int               fd,
                 DBusError        *error);
int 
_dbus_read      (int               fd,
                 DBusString       *buffer,
                 int               count);
int 
_dbus_write     (int               fd,
                 const DBusString *buffer,
                 int               start,
                 int               len);
int 
_dbus_write_two (int               fd,
                 const DBusString *buffer1,
                 int               start1,
                 int               len1,
                 const DBusString *buffer2,
                 int               start2,
                 int               len2);

dbus_bool_t _dbus_open_unix_socket (int              *fd,
                                    DBusError        *error);
int _dbus_connect_unix_socket (const char     *path,
                               dbus_bool_t     abstract,
                               DBusError      *error);
int _dbus_listen_unix_socket  (const char     *path,
                               dbus_bool_t     abstract,
                               DBusError      *error);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_SYSDEPS_UNIX_H */
