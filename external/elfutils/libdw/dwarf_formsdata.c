
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formsdata (attr, return_sval)
     Dwarf_Attribute *attr;
     Dwarf_Sword *return_sval;
{
  if (attr == NULL)
    return -1;

  unsigned int u128;
  unsigned char *datap;

  switch (attr->form)
    {
    case DW_FORM_data1:
      *return_sval = *attr->valp;
      break;

    case DW_FORM_data2:
      *return_sval = read_2ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_data4:
      *return_sval = read_4ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_data8:
      *return_sval = read_8ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_sdata:
      datap = attr->valp;
      get_sleb128 (u128, datap);
      *return_sval = u128;
      break;

    case DW_FORM_udata:
      datap = attr->valp;
      get_uleb128 (u128, datap);
      *return_sval = u128;
      break;

    default:
      __libdw_seterrno (DWARF_E_NO_CONSTANT);
      return -1;
    }

  return 0;
}
