
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdint.h>
#include <libeblP.h>


bool
ebl_debugscn_p (ebl, name)
     Ebl *ebl;
     const char *name;
{
  return ebl->debugscn_p (name);
}
