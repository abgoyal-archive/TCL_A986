
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_getattrcnt (abbrev, attrcntp)
     Dwarf_Abbrev *abbrev;
     size_t *attrcntp;
{
  if (abbrev == NULL)
    return -1;

  *attrcntp = abbrev->attrcnt;

  return 0;
}
