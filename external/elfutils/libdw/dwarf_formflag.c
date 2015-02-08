
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formflag (attr, return_bool)
     Dwarf_Attribute *attr;
     bool *return_bool;
{
  if (attr == NULL)
    return -1;

  if (unlikely (attr->form != DW_FORM_flag))
    {
      __libdw_seterrno (DWARF_E_NO_FLAG);
      return -1;
    }

  *return_bool = *attr->valp != 0;

  return 0;
}
