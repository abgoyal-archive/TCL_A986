
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"
#include <string.h>


int
dwarf_haschildren (die)
     Dwarf_Die *die;
{
  /* Find the abbreviation entry.  */
  Dwarf_Abbrev *abbrevp = die->abbrev;
  if (abbrevp != (Dwarf_Abbrev *) -1l)
    {
      unsigned char *readp = (unsigned char *) die->addr;

      /* First we have to get the abbreviation code so that we can decode
	 the data in the DIE.  */
      unsigned int abbrev_code;
      get_uleb128 (abbrev_code, readp);

      abbrevp = __libdw_findabbrev (die->cu, abbrev_code);
      die->abbrev = abbrevp ?: (Dwarf_Abbrev *) -1l;
    }
  if (unlikely (die->abbrev == (Dwarf_Abbrev *) -1l))
    {
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      return 0;
    }

  return die->abbrev->has_children;
}
