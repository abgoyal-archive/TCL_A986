
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


char *
elf_rawfile (elf, ptr)
     Elf *elf;
     size_t *ptr;
{
  if (elf == NULL)
    {
      /* No valid descriptor.  */
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
    error_out:
      if (ptr != NULL)
	*ptr = 0;
      return NULL;
    }

  /* If the file is not mmap'ed and not previously loaded, do it now.  */
  if (elf->map_address == NULL && __libelf_readall (elf) == NULL)
    goto error_out;

  if (ptr != NULL)
    *ptr = elf->maximum_size;

  return (char *) elf->map_address + elf->start_offset;
}
