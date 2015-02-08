
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stddef.h>

#include "libelfP.h"


char *
elf_getident (elf, ptr)
     Elf *elf;
     size_t *ptr;
{
  /* In case this is no ELF file, the handle is invalid and we return
     NULL.  */
  if (elf == NULL || elf->kind != ELF_K_ELF)
    {
      if (ptr != NULL)
	*ptr = 0;
      return NULL;
    }

  /* We already read the ELF header.  Return a pointer to it and store
     the length in *PTR.  */
  if (ptr != NULL)
    *ptr = EI_NIDENT;

  return (char *) (elf->class == ELFCLASS32
		   || (offsetof (struct Elf, state.elf32.ehdr)
		       == offsetof (struct Elf, state.elf64.ehdr))
		   ? elf->state.elf32.ehdr->e_ident
		   : elf->state.elf64.ehdr->e_ident);
}
