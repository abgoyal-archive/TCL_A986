
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


int
dwarf_formblock (attr, return_block)
     Dwarf_Attribute *attr;
     Dwarf_Block *return_block;
{
  if (attr == NULL)
    return -1;

  unsigned int u128;
  unsigned char *datap;

  switch (attr->form)
    {
    case DW_FORM_block1:
      return_block->length = *(uint8_t *) attr->valp;
      return_block->data = attr->valp + 1;
      break;

    case DW_FORM_block2:
      return_block->length = read_2ubyte_unaligned (attr->cu->dbg, attr->valp);
      return_block->data = attr->valp + 2;
      break;

    case DW_FORM_block4:
      return_block->length = read_4ubyte_unaligned (attr->cu->dbg, attr->valp);
      return_block->data = attr->valp + 4;
      break;

    case DW_FORM_block:
      datap = attr->valp;
      get_uleb128 (u128, datap);
      return_block->length = u128;
      return_block->data = datap;
      break;

    default:
      __libdw_seterrno (DWARF_E_NO_BLOCK);
      return -1;
    }

  if (return_block->data + return_block->length
      > ((unsigned char *) attr->cu->dbg->sectiondata[IDX_debug_info]->d_buf
	 + attr->cu->dbg->sectiondata[IDX_debug_info]->d_size))
    {
      /* Block does not fit.  */
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      return -1;
    }

  return 0;
}
