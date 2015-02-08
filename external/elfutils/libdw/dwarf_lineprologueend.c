
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_lineprologueend (Dwarf_Line *line, bool *flagp)
{
  if (line == NULL)
    return -1;

  *flagp =  line->prologue_end;

  return 0;
}
