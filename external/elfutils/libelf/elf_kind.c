
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf_Kind
elf_kind (elf)
     Elf *elf;
{
  return elf == NULL ? ELF_K_NONE : elf->kind;
}
