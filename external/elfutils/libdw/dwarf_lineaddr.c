
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_lineaddr (Dwarf_Line *line, Dwarf_Addr *addrp)
{
  if (line == NULL)
    return -1;

  *addrp =  line->addr;

  return 0;
}
