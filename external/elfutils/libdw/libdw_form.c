
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include <string.h>

#include "libdwP.h"


size_t
internal_function_def
__libdw_form_val_len (Dwarf *dbg, struct Dwarf_CU *cu, unsigned int form,
		      unsigned char *valp)
{
  unsigned char *saved;
  unsigned int u128;
  size_t result;

  switch (form)
    {
    case DW_FORM_addr:
      result = cu->address_size;
      break;

    case DW_FORM_strp:
    case DW_FORM_ref_addr:
      result = cu->offset_size;
      break;

    case DW_FORM_block1:
      result = *valp + 1;
      break;

    case DW_FORM_block2:
      result = read_2ubyte_unaligned (dbg, valp) + 2;
      break;

    case DW_FORM_block4:
      result = read_4ubyte_unaligned (dbg, valp) + 4;
      break;

    case DW_FORM_block:
      saved = valp;
      get_uleb128 (u128, valp);
      result = u128 + (valp - saved);
      break;

    case DW_FORM_ref1:
    case DW_FORM_data1:
    case DW_FORM_flag:
      result = 1;
      break;

    case DW_FORM_data2:
    case DW_FORM_ref2:
      result = 2;
      break;

    case DW_FORM_data4:
    case DW_FORM_ref4:
      result = 4;
      break;

    case DW_FORM_data8:
    case DW_FORM_ref8:
      result = 8;
      break;

    case DW_FORM_string:
      result = strlen ((char *) valp) + 1;
      break;

    case DW_FORM_sdata:
    case DW_FORM_udata:
    case DW_FORM_ref_udata:
      saved = valp;
      get_uleb128 (u128, valp);
      result = valp - saved;
      break;

    case DW_FORM_indirect:
      saved = valp;
      get_uleb128 (u128, valp);
      // XXX Is this really correct?
      result = __libdw_form_val_len (dbg, cu, u128, valp);
      if (result != (size_t) -1)
	result += valp - saved;
      break;

    default:
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      result = (size_t) -1l;
      break;
    }

  return result;
}
