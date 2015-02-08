
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdlib.h>
#include <string.h>

#include "libelfP.h"

#ifndef LIBELFBITS
# define LIBELFBITS 32
#endif


ElfW2(LIBELFBITS,Ehdr) *
elfw2(LIBELFBITS,newehdr) (elf)
     Elf *elf;
{
  ElfW2(LIBELFBITS,Ehdr) *result;

  if (elf == NULL)
    return NULL;

  if (unlikely (elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  rwlock_wrlock (elf->lock);

  if (elf->class == 0)
    elf->class = ELFW(ELFCLASS,LIBELFBITS);
  else if (unlikely (elf->class != ELFW(ELFCLASS,LIBELFBITS)))
    {
      __libelf_seterrno (ELF_E_INVALID_CLASS);
      result = NULL;
      goto out;
    }

  /* Don't create an ELF header if one already exists.  */
  if (elf->state.ELFW(elf,LIBELFBITS).ehdr == NULL)
    {
      /* We use the memory in the ELF descriptor.  */
      elf->state.ELFW(elf,LIBELFBITS).ehdr =
	&elf->state.ELFW(elf,LIBELFBITS).ehdr_mem;

      /* We clear this memory.  */
      memset (elf->state.ELFW(elf,LIBELFBITS).ehdr, '\0',
	      sizeof (ElfW2(LIBELFBITS,Ehdr)));

      /* Mark the ELF header has modified.  */
      elf->state.ELFW(elf,LIBELFBITS).ehdr_flags |= ELF_F_DIRTY;
    }

  result = elf->state.ELFW(elf,LIBELFBITS).ehdr;

 out:
  rwlock_unlock (elf->lock);

  return result;
}
INTDEF(elfw2(LIBELFBITS,newehdr))
