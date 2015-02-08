
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <libeblP.h>


const char *
ebl_backend_name (ebl)
     Ebl *ebl;
{
  return ebl != NULL ? ebl->emulation : gettext ("No backend");
}
