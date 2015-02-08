
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formaddr (attr, return_addr)
     Dwarf_Attribute *attr;
     Dwarf_Addr *return_addr;
{
  if (attr == NULL)
    return -1;

  if (unlikely (attr->form != DW_FORM_addr))
    {
      __libdw_seterrno (DWARF_E_NO_ADDR);
      return -1;
    }

  if (attr->cu->address_size == 8)
    *return_addr = read_8ubyte_unaligned (attr->cu->dbg, attr->valp);
  else
    *return_addr = read_4ubyte_unaligned (attr->cu->dbg, attr->valp);

  return 0;
}
