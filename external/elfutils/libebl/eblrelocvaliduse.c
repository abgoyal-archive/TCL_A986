
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libeblP.h>


bool
ebl_reloc_valid_use (ebl, reloc)
     Ebl *ebl;
     int reloc;
{
  return ebl != NULL ? ebl->reloc_valid_use (ebl->elf, reloc) : false;
}
