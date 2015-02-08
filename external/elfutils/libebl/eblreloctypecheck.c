
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libeblP.h>


bool
ebl_reloc_type_check (ebl, reloc)
     Ebl *ebl;
     int reloc;
{
  return ebl != NULL ? ebl->reloc_type_check (reloc) : false;
}
