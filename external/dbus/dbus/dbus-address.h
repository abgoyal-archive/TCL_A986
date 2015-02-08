
/* -*- mode: C; c-file-style: "gnu" -*- */
#if !defined (DBUS_INSIDE_DBUS_H) && !defined (DBUS_COMPILATION)
#error "Only <dbus/dbus.h> can be included directly, this file may disappear or change contents."
#endif

#ifndef DBUS_ADDRESS_H
#define DBUS_ADDRESS_H

#include <dbus/dbus-types.h>
#include <dbus/dbus-errors.h>

DBUS_BEGIN_DECLS


/** Opaque type representing one of the semicolon-separated items in an address */
typedef struct DBusAddressEntry DBusAddressEntry;

dbus_bool_t dbus_parse_address            (const char         *address,
					   DBusAddressEntry ***entry,
					   int                *array_len,
					   DBusError          *error);
const char *dbus_address_entry_get_value  (DBusAddressEntry   *entry,
					   const char         *key);
const char *dbus_address_entry_get_method (DBusAddressEntry   *entry);
void        dbus_address_entries_free     (DBusAddressEntry  **entries);

char* dbus_address_escape_value   (const char *value);
char* dbus_address_unescape_value (const char *value,
                                   DBusError  *error);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_ADDRESS_H */

