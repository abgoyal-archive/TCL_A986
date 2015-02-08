
#include <config.h>
#include "system.h"

#include <quotearg.h>

#include "location.h"

location const empty_location;

void
location_print (FILE *out, location loc)
{
  fprintf (out, "%s:%d.%d",
	   quotearg_n_style (3, escape_quoting_style, loc.start.file),
	   loc.start.line, loc.start.column);

  if (loc.start.file != loc.end.file)
    fprintf (out, "-%s:%d.%d",
	     quotearg_n_style (3, escape_quoting_style, loc.end.file),
	     loc.end.line, loc.end.column - 1);
  else if (loc.start.line < loc.end.line)
    fprintf (out, "-%d.%d", loc.end.line, loc.end.column - 1);
  else if (loc.start.column < loc.end.column - 1)
    fprintf (out, "-%d", loc.end.column - 1);
}
