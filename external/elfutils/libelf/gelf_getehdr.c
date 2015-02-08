
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <string.h>

#include "libelfP.h"


GElf_Ehdr *
gelf_getehdr (elf, dest)
     Elf *elf;
     GElf_Ehdr *dest;
{
  GElf_Ehdr *result = NULL;

  if (elf == NULL)
    return NULL;

  if (unlikely (elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  rwlock_rdlock (elf->lock);

  if (elf->class == ELFCLASS32)
    {
      Elf32_Ehdr *ehdr = elf->state.elf32.ehdr;

      /* Maybe no ELF header was created yet.  */
      if (ehdr == NULL)
	__libelf_seterrno (ELF_E_WRONG_ORDER_EHDR);
      else
	{
	  /* Convert the 32-bit struct to an 64-bit one.  */
	  memcpy (dest->e_ident, ehdr->e_ident, EI_NIDENT);
#define COPY(name) \
	  dest->name = ehdr->name
	  COPY (e_type);
	  COPY (e_machine);
	  COPY (e_version);
	  COPY (e_entry);
	  COPY (e_phoff);
	  COPY (e_shoff);
	  COPY (e_flags);
	  COPY (e_ehsize);
	  COPY (e_phentsize);
	  COPY (e_phnum);
	  COPY (e_shentsize);
	  COPY (e_shnum);
	  COPY (e_shstrndx);

	  result = dest;
	}
    }
  else
    {
      /* Maybe no ELF header was created yet.  */
      if (elf->state.elf64.ehdr == NULL)
	__libelf_seterrno (ELF_E_WRONG_ORDER_EHDR);
      else
	result = memcpy (dest, elf->state.elf64.ehdr, sizeof (*dest));
    }

  rwlock_unlock (elf->lock);

  return result;
}
