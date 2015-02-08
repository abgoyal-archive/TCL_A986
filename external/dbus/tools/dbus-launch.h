
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_LAUNCH_H
#define DBUS_LAUNCH_H

#include <config.h>
#include <sys/types.h>

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#undef  MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#define MAX_ADDR_LEN 512

/* defined in dbus-launch.c */
void verbose (const char *format, ...);
char *xstrdup (const char *str);
void kill_bus_and_exit (int exitcode);

const char* get_machine_uuid (void);

#ifdef DBUS_BUILD_X11
/* defined in dbus-launch-x11.c */
int x11_init (void);
int x11_get_address (char **paddress, pid_t *pid, long *wid);
int x11_save_address (char *address, pid_t pid, long *wid);
void x11_handle_event (void);
#endif

#endif
