
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


int
dwarf_offabbrev (Dwarf *dbg, Dwarf_Off offset, size_t *lengthp,
		 Dwarf_Abbrev *abbrevp)
{
  if (dbg == NULL)
    return -1;

  return __libdw_getabbrev (dbg, NULL, offset, lengthp, abbrevp) == NULL;
}
