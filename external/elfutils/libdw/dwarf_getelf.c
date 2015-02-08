
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stddef.h>

#include "libdwP.h"


Elf *
dwarf_get_elf (dwarf)
     Dwarf *dwarf;
{
  if (dwarf == NULL)
    /* Some error occurred before.  */
    return NULL;

  return dwarf->elf;
}
