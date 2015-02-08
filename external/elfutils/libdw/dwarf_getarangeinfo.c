
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libdwP.h>


int
dwarf_getarangeinfo (Dwarf_Arange *arange, Dwarf_Addr *addrp,
		     Dwarf_Word *lengthp, Dwarf_Off *offsetp)
{
  if (arange == NULL)
    return -1;

  if (addrp != NULL)
    *addrp = arange->addr;
  if (lengthp != NULL)
    *lengthp = arange->length;
  if (offsetp != NULL)
    *offsetp = arange->offset;

  return 0;
}
