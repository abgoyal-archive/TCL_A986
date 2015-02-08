
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


size_t
elf_ndxscn (scn)
     Elf_Scn *scn;
{
  if (scn == NULL)
    return SHN_UNDEF;

  return scn->index;
}
