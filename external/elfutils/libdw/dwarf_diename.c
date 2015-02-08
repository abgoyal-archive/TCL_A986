
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


const char *
dwarf_diename (die)
     Dwarf_Die *die;
{
  Dwarf_Attribute attr_mem;

  return dwarf_formstring (dwarf_attr (die, DW_AT_name, &attr_mem));
}
