
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>
#include "libdwP.h"


Dwarf_Die *
dwarf_offdie (dbg, offset, result)
     Dwarf *dbg;
     Dwarf_Off offset;
     Dwarf_Die *result;
{
  if (dbg == NULL)
    return NULL;

  if (offset >= dbg->sectiondata[IDX_debug_info]->d_size)
    {
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      return NULL;
    }

  /* Clear the entire DIE structure.  This signals we have not yet
     determined any of the information.  */
  memset (result, '\0', sizeof (Dwarf_Die));

  result->addr = (char *) dbg->sectiondata[IDX_debug_info]->d_buf + offset;

  /* Get the CU.  */
  result->cu = __libdw_findcu (dbg, offset);
  if (result->cu == NULL)
    {
      /* This should never happen.  The input file is malformed.  */
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      result = NULL;
    }

  return result;
}
