
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


unsigned int
dwarf_whatform (attr)
     Dwarf_Attribute *attr;
{
  return attr == NULL ? 0 : attr->form;
}
