
/* -*- mode: C; c-file-style: "gnu" -*- */

#include <config.h>

#include <dbus/dbus-internals.h>
#include "dir-watch.h"


/* NoOp */

void 
bus_drop_all_directory_watches (void)
{
}

void
bus_watch_directory (const char *dir, BusContext *context)
{
}
