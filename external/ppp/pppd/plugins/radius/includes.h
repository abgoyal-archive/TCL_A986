

#include <sys/types.h>

#include <ctype.h>
#include <stdio.h>
#include <errno.h>
#include <netdb.h>
#include <syslog.h>

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <limits.h>

#ifndef PATH_MAX
#define PATH_MAX        1024
#endif

#ifndef UCHAR_MAX
# define UCHAR_MAX       255
#endif

#include <pwd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <time.h>

#include "magic.h"

/* rlib/lock.c */
int do_lock_exclusive(int);
int do_unlock(int);
