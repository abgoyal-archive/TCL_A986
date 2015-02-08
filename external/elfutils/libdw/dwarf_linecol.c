
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_linecol (Dwarf_Line *line, int *colp)
{
  if (line == NULL)
    return -1;

  *colp =  line->column;

  return 0;
}
