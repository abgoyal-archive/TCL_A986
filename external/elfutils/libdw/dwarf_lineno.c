
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_lineno (Dwarf_Line *line, int *linep)
{
  if (line == NULL)
    return -1;

  *linep =  line->line;

  return 0;
}
