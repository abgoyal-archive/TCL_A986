
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


const char *
dwarf_filesrc (Dwarf_Files *file, size_t idx, Dwarf_Word *mtime,
	       Dwarf_Word *length)
{
  if (file == NULL || idx >= file->nfiles)
    return NULL;

  if (mtime != NULL)
    *mtime = file->info[idx].mtime;

  if (length != NULL)
    *length = file->info[idx].length;

  return file->info[idx].name;
}
