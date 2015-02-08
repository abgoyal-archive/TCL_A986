
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libeblP.h>


GElf_Word
ebl_sh_flags_combine (ebl, flags1, flags2)
     Ebl *ebl;
     GElf_Word flags1;
     GElf_Word flags2;
{
  return ebl->sh_flags_combine (flags1, flags2);
}
