
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


Dwarf_Off
dwarf_dieoffset (die)
     Dwarf_Die *die;
{
  return (die == NULL
	  ? -1l
	  : die->addr - die->cu->dbg->sectiondata[IDX_debug_info]->d_buf);
}
