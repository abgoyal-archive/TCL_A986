
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>
#include <stdlib.h>
#include <unistd.h>

#include "libelfP.h"


char *
gelf_rawchunk (elf, offset, size)
     Elf *elf;
     GElf_Off offset;
     GElf_Word size;
{
  if (elf == NULL)
    {
      /* No valid descriptor.  */
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  if (offset >= elf->maximum_size
      || offset + size >= elf->maximum_size
      || offset + size < offset)
    {
      /* Invalid request.  */
      __libelf_seterrno (ELF_E_INVALID_OP);
      return NULL;
    }

  /* If the file is mmap'ed return an appropriate pointer.  */
  if (elf->map_address != NULL)
    return (char *) elf->map_address + elf->start_offset + offset;

  /* We allocate the memory and read the data from the file.  */
  char *result = (char *) malloc (size);
  if (result == NULL)
    __libelf_seterrno (ELF_E_NOMEM);
  else
    /* Read the file content.  */
    if ((size_t) pread (elf->fildes, result, size, elf->start_offset + offset)
	!= size)
      {
	/* Something went wrong.  */
	__libelf_seterrno (ELF_E_READ_ERROR);
	free (result);
      }

  return result;
}
