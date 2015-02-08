
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_highpc (die, return_addr)
     Dwarf_Die *die;
     Dwarf_Addr *return_addr;
{
  Dwarf_Attribute attr_mem;

  return dwarf_formaddr (dwarf_attr (die, DW_AT_high_pc, &attr_mem),
			 return_addr);
}
