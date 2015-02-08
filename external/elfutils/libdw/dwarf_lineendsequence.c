
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_lineendsequence (Dwarf_Line *line, bool *flagp)
{
  if (line == NULL)
    return -1;

  *flagp =  line->end_sequence;

  return 0;
}
