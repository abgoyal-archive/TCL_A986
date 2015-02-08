
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formudata (attr, return_uval)
     Dwarf_Attribute *attr;
     Dwarf_Word *return_uval;
{
  if (attr == NULL)
    return -1;

  unsigned int u128;
  unsigned char *datap;

  switch (attr->form)
    {
    case DW_FORM_data1:
      *return_uval = *attr->valp;
      break;

    case DW_FORM_data2:
      *return_uval = read_2ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_data4:
      *return_uval = read_4ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_data8:
      *return_uval = read_8ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_sdata:
      datap = attr->valp;
      get_sleb128 (u128, datap);
      *return_uval = u128;
      break;

    case DW_FORM_udata:
      datap = attr->valp;
      get_uleb128 (u128, datap);
      *return_uval = u128;
      break;

    default:
      __libdw_seterrno (DWARF_E_NO_CONSTANT);
      return -1;
    }

  return 0;
}
