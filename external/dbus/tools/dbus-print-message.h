
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_PRINT_MESSAGE_H
#define DBUS_PRINT_MESSAGE_H

#include <stdio.h>
#include <string.h>
#include <dbus/dbus.h>

void print_message (DBusMessage *message, dbus_bool_t literal);

#endif /* DBUS_PRINT_MESSAGE_H */
