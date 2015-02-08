
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_hasattr (die, search_name)
     Dwarf_Die *die;
     unsigned int search_name;
{
  if (die == NULL)
    return 0;

  /* Search for the attribute with the given name.  */
  unsigned int code;
  (void) __libdw_find_attr (die, search_name, &code, NULL);

  return code == search_name;
}
