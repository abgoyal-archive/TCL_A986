
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <dwarf.h>
#include "libdwP.h"


const char *
dwarf_formstring (attrp)
     Dwarf_Attribute *attrp;
{
  /* Ignore earlier errors.  */
  if (attrp == NULL)
    return NULL;

  /* We found it.  Now determine where the string is stored.  */
  if (attrp->form == DW_FORM_string)
    /* A simple inlined string.  */
    return (const char *) attrp->valp;

  Dwarf *dbg = attrp->cu->dbg;

  if (unlikely (attrp->form != DW_FORM_strp)
      || dbg->sectiondata[IDX_debug_str] == NULL)
    {
    invalid_error:
      __libdw_seterrno (DWARF_E_NO_STRING);
      return NULL;
    }

  uint64_t off;
  // XXX We need better boundary checks.
  if (attrp->cu->offset_size == 8)
    off = read_8ubyte_unaligned (dbg, attrp->valp);
  else
    off = read_4ubyte_unaligned (dbg, attrp->valp);

  if (off  >= dbg->sectiondata[IDX_debug_str]->d_size)
    goto invalid_error;

  return (const char *) dbg->sectiondata[IDX_debug_str]->d_buf + off;
}
