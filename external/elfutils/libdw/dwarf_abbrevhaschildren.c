
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <dwarf.h>
#include "libdwP.h"


int
dwarf_abbrevhaschildren (abbrev)
     Dwarf_Abbrev *abbrev;
{
  return abbrev == NULL ? -1 : abbrev->has_children;
}
