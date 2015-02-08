
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <dwarf.h>
#include "libdwP.h"


unsigned int
dwarf_getabbrevtag (abbrev)
     Dwarf_Abbrev *abbrev;
{
  return abbrev == NULL ? 0 : abbrev->tag;
}
