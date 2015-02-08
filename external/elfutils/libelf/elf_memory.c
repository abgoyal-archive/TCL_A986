
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf *
elf_memory (image, size)
     char *image;
     size_t size;
{
  if (image == NULL)
    {
      __libelf_seterrno (ELF_E_INVALID_OPERAND);
      return NULL;
    }

  return __libelf_read_mmaped_file (-1, image, 0, size, ELF_C_READ, NULL);
}
