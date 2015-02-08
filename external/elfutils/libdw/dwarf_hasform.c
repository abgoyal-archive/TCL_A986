
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_hasform (attr, search_form)
     Dwarf_Attribute *attr;
     unsigned int search_form;
{
  if (attr == NULL)
    return 0;

  return attr->form == search_form;
}
