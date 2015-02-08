
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


Dwarf_Attribute *
dwarf_attr (die, search_name, result)
     Dwarf_Die *die;
     unsigned int search_name;
     Dwarf_Attribute *result;
{
  if (die == NULL)
    return NULL;

  /* Search for the attribute with the given name.  */
  result->valp = __libdw_find_attr (die, search_name, &result->code,
				    &result->form);
  /* Always fill in the CU information.  */
  result->cu = die->cu;

  return result->code == search_name ? result : NULL;
}
