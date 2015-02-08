
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


Dwarf_Line *
dwarf_onesrcline (Dwarf_Lines *lines, size_t idx)
{
  if (lines == NULL)
    return NULL;

  if (idx >= lines->nlines)
    {
      __libdw_seterrno (DWARF_E_INVALID_LINE_IDX);
      return NULL;
    }

  return &lines->info[idx];
}
