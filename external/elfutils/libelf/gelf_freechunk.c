
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>
#include <stdlib.h>

#include "libelfP.h"


void
gelf_freechunk (elf, ptr)
     Elf *elf;
     char *ptr;
{
  if (elf == NULL)
    /* No valid descriptor.  */
    return;

  /* We do not have to do anything if the pointer returned by
     gelf_rawchunk points into the memory allocated for the ELF
     descriptor.  */
  if (ptr < (char *) elf->map_address + elf->start_offset
      || ptr >= ((char *) elf->map_address + elf->start_offset
		 + elf->maximum_size))
    free (ptr);
}
