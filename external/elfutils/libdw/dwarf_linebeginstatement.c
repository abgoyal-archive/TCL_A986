
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_linebeginstatement (Dwarf_Line *line, bool *flagp)
{
  if (line == NULL)
    return -1;

  *flagp =  line->is_stmt;

  return 0;
}
