
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_reloc_type_name (ebl, reloc, buf, len)
     Ebl *ebl;
     int reloc;
     char *buf;
     size_t len;
{
  const char *res;

  res = ebl != NULL ? ebl->reloc_type_name (reloc, buf, len) : NULL;
  if (res == NULL)
    /* There are no generic relocation type names.  */
    res = "???";

  return res;
}
