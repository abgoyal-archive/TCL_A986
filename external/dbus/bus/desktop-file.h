
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef BUS_DESKTOP_FILE_H
#define BUS_DESKTOP_FILE_H

#include <dbus/dbus.h>
#include <dbus/dbus-string.h>

#define BUS_DESKTOP_PARSE_ERROR_INVALID_SYNTAX  "org.freedesktop.DBus.DesktopParseError.InvalidSyntax"
#define BUS_DESKTOP_PARSE_ERROR_INVALID_ESCAPES "org.freedesktop.DBus.DesktopParseError.InvalidEscapes"
#define BUS_DESKTOP_PARSE_ERROR_INVALID_CHARS   "org.freedesktop.DBus.DesktopParseError.InvalidChars"

typedef struct BusDesktopFile BusDesktopFile;

BusDesktopFile *bus_desktop_file_load (DBusString     *filename,
				       DBusError      *error);
void            bus_desktop_file_free (BusDesktopFile *file);

dbus_bool_t bus_desktop_file_get_raw    (BusDesktopFile  *desktop_file,
					 const char      *section_name,
					 const char      *keyname,
					 const char     **val);
dbus_bool_t bus_desktop_file_get_string (BusDesktopFile  *desktop_file,
					 const char      *section,
					 const char      *keyname,
					 char           **val,
					 DBusError       *error);


#endif /* BUS_DESKTOP_FILE_H */
