

#include <config.h>
#include "system.h"

#include <stdarg.h>

#include "complain.h"
#include "files.h"

extern char *program_name;

/* This variable is set each time `warn' is called.  */
bool warning_issued;

/* This variable is set each time `complain' is called.  */
bool complaint_issued;



void
warn_at (location loc, const char *message, ...)
{
  va_list args;

  location_print (stderr, loc);
  fputs (": ", stderr);
  fputs (_("warning: "), stderr);

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);

  warning_issued = true;
  putc ('\n', stderr);
}

void
warn (const char *message, ...)
{
  va_list args;

  fprintf (stderr, "%s: %s", current_file ? current_file : program_name, _("warning: "));

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);

  warning_issued = true;
  putc ('\n', stderr);
}


void
complain_at (location loc, const char *message, ...)
{
  va_list args;

  location_print (stderr, loc);
  fputs (": ", stderr);

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);

  complaint_issued = true;
  putc ('\n', stderr);
}

void
complain (const char *message, ...)
{
  va_list args;

  fprintf (stderr, "%s: ", current_file ? current_file : program_name);

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);

  complaint_issued = true;
  putc ('\n', stderr);
}


void
fatal_at (location loc, const char *message, ...)
{
  va_list args;

  location_print (stderr, loc);
  fputs (": ", stderr);
  fputs (_("fatal error: "), stderr);

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);
  putc ('\n', stderr);
  exit (EXIT_FAILURE);
}

void
fatal (const char *message, ...)
{
  va_list args;

  fprintf (stderr, "%s: ", current_file ? current_file : program_name);

  fputs (_("fatal error: "), stderr);

  va_start (args, message);
  vfprintf (stderr, message, args);
  va_end (args);
  putc ('\n', stderr);
  exit (EXIT_FAILURE);
}
