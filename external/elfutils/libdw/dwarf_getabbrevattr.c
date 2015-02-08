
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <dwarf.h>
#include "libdwP.h"


int
dwarf_getabbrevattr (abbrev, idx, namep, formp, offsetp)
     Dwarf_Abbrev *abbrev;
     size_t idx;
     unsigned int *namep;
     unsigned int *formp;
     Dwarf_Off *offsetp;
{
  if (abbrev == NULL)
    return -1;

  size_t cnt = 0;
  unsigned char *attrp = abbrev->attrp;
  unsigned char *start_attrp;
  unsigned int name;
  unsigned int form;

  do
    {
      start_attrp = attrp;

      /* Attribute code and form are encoded as ULEB128 values.  */
      get_uleb128 (name, attrp);
      get_uleb128 (form, attrp);

      /* If both values are zero the index is out of range.  */
      if (name == 0 && form == 0)
	return -1;
    }
  while (cnt++ < idx);

  /* Store the result if requested.  */
  if (namep != NULL)
    *namep = name;
  if (formp != NULL)
    *formp = form;
  if (offsetp != NULL)
    *offsetp = (start_attrp - abbrev->attrp) + abbrev->offset;

  return 0;
}
