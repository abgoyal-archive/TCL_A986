
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libeblP.h>


bool
ebl_gotpc_reloc_check (ebl, reloc)
     Ebl *ebl;
     int reloc;
{
  return ebl != NULL ? ebl->gotpc_reloc_check (ebl->elf, reloc) : false;
}
