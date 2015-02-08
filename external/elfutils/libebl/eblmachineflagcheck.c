
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libeblP.h>


bool
ebl_machine_flag_check (ebl, flags)
     Ebl *ebl;
     Elf64_Word flags;
{
  return ebl != NULL ? ebl->machine_flag_check (flags) : (flags == 0);
}
