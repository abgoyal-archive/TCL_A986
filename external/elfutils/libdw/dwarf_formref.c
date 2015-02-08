
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formref (attr, return_offset)
     Dwarf_Attribute *attr;
     Dwarf_Off *return_offset;
{
  if (attr == NULL)
    return -1;

  unsigned int u128;
  unsigned char *datap;

  switch (attr->form)
    {
    case DW_FORM_ref1:
      *return_offset = *attr->valp;
      break;

    case DW_FORM_ref2:
      *return_offset = read_2ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_ref4:
      *return_offset = read_4ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_ref8:
      *return_offset = read_8ubyte_unaligned (attr->cu->dbg, attr->valp);
      break;

    case DW_FORM_ref_udata:
      datap = attr->valp;
      get_uleb128 (u128, datap);
      *return_offset = u128;
      break;

    case DW_FORM_ref_addr:
      __libdw_seterrno (DWARF_E_INVALID_REFERENCE);
      return -1;

    default:
      __libdw_seterrno (DWARF_E_NO_REFERENCE);
      return -1;
    }

  return 0;
}
