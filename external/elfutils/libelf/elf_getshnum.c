
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <stddef.h>

#include "libelfP.h"


int
elf_getshnum (elf, dst)
     Elf *elf;
     size_t *dst;
{
  int result = 0;
  int idx;

  if (elf == NULL)
    return -1;

  if (unlikely (elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return -1;
    }

  rwlock_rdlock (elf->lock);

  idx = elf->state.elf.scns_last->cnt;
  if (idx != 0
      || (elf->state.elf.scns_last
	  != (elf->class == ELFCLASS32
	      || (offsetof (Elf, state.elf32.scns)
		  == offsetof (Elf, state.elf64.scns))
	      ? &elf->state.elf32.scns : &elf->state.elf64.scns)))
    /* There is at least one section.  */
    *dst = 1 + elf->state.elf.scns_last->data[idx - 1].index;
  else
    *dst = 0;

  rwlock_unlock (elf->lock);

  return result;
}
INTDEF(elf_getshnum)
