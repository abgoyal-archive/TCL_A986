
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


Dwarf_Line *
dwarf_getsrc_die (Dwarf_Die *cudie, Dwarf_Addr addr)
{
  Dwarf_Lines *lines;
  size_t nlines;

  if (dwarf_getsrclines (cudie, &lines, &nlines) != 0)
    return NULL;

  if (addr >= lines->info[0].addr )
    for (size_t n = 1; n < nlines; ++n)
      if (addr < lines->info[n].addr)
	return &lines->info[n - 1];

  __libdw_seterrno (DWARF_E_ADDR_OUTOFRANGE);
  return NULL;
}
