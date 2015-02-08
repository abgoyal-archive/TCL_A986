
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


const char *
dwarf_linesrc (Dwarf_Line *line, Dwarf_Word *mtime, Dwarf_Word *length)
{
  if (line == NULL)
    return NULL;

  if (line->file >= line->files->nfiles)
    {
      __libdw_seterrno (DWARF_E_INVALID_DWARF);
      return NULL;
    }

  if (mtime != NULL)
    *mtime = line->files->info[line->file].mtime;

  if (length != NULL)
    *length = line->files->info[line->file].length;

  return line->files->info[line->file].name;
}
