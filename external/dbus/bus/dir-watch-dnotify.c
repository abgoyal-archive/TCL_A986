
/* -*- mode: C; c-file-style: "gnu" -*- */

#include <config.h>

#define _GNU_SOURCE
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <dbus/dbus-internals.h>
#include "dir-watch.h"

#define MAX_DIRS_TO_WATCH 128

/* use a static array to avoid handling OOM */
static int fds[MAX_DIRS_TO_WATCH];
static int num_fds = 0;

void
bus_watch_directory (const char *dir, BusContext *context)
{
  int fd;

  _dbus_assert (dir != NULL);

  if (num_fds >= MAX_DIRS_TO_WATCH )
    {
      _dbus_warn ("Cannot watch config directory '%s'. Already watching %d directories\n", dir, MAX_DIRS_TO_WATCH);
      goto out;
    }

  fd = open (dir, O_RDONLY);
  if (fd < 0)
    {
      _dbus_warn ("Cannot open directory '%s'; error '%s'\n", dir, _dbus_strerror (errno));
      goto out;
    }

  if (fcntl (fd, F_NOTIFY, DN_DELETE|DN_RENAME|DN_MODIFY) == -1)
    {
      _dbus_warn ("Cannot setup D_NOTIFY for '%s' error '%s'\n", dir, _dbus_strerror (errno));
      close (fd);
      goto out;
    }
  
  fds[num_fds++] = fd;
  _dbus_verbose ("Added watch on config directory '%s'\n", dir);

 out:
  ;
}

void 
bus_drop_all_directory_watches (void)
{
  int i;
 
  _dbus_verbose ("Dropping all watches on config directories\n");
 
  for (i = 0; i < num_fds; i++)
    {
      if (close (fds[i]) != 0)
	{
	  _dbus_verbose ("Error closing fd %d for config directory watch\n", fds[i]);
	}
    }
  
  num_fds = 0;
}
