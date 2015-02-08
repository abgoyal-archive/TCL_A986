
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>
#include "libdwP.h"


const char *
dwarf_getstring (dbg, offset, lenp)
     Dwarf *dbg;
     Dwarf_Off offset;
     size_t *lenp;
{
  if (dbg == NULL)
    return NULL;

  if (dbg->sectiondata[IDX_debug_str] == NULL
      || offset >= dbg->sectiondata[IDX_debug_str]->d_size)
    {
    no_string:
      __libdw_seterrno (DWARF_E_NO_STRING);
      return NULL;
    }

  const char *result = ((const char *) dbg->sectiondata[IDX_debug_str]->d_buf
			+ offset);
  const char *endp = memchr (result, '\0',
			     dbg->sectiondata[IDX_debug_str]->d_size - offset);
  if (endp == NULL)
    goto no_string;

  if (lenp != NULL)
    *lenp = endp - result;

  return result;
}
