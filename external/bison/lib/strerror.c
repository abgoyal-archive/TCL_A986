
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <limits.h>

int sprintf (char *, char const *, ...);
extern int sys_nerr;
extern char *sys_errlist[];

char *
strerror (int n)
{
  static char const fmt[] = "Unknown error (%d)";
  static char mesg[sizeof fmt + sizeof n * CHAR_BIT / 3];

  if (n < 0 || n >= sys_nerr)
    {
      sprintf (mesg, fmt, n);
      return mesg;
    }
  else
    return sys_errlist[n];
}
