
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "bus.h"

#ifndef DIR_WATCH_H
#define DIR_WATCH_H

/* setup a watch on a directory (OS dependent, may be a NOP) */
void bus_watch_directory (const char *directory, BusContext *context);

/* drop all the watches previously set up by bus_config_watch_directory (OS dependent, may be a NOP) */
void bus_drop_all_directory_watches (void);

#endif /* DIR_WATCH_H */
