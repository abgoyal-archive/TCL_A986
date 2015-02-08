
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <stddef.h>

#include "libelfP.h"


int
gelf_getclass (elf)
     Elf *elf;
{
  return elf == NULL || elf->kind != ELF_K_ELF ? ELFCLASSNONE : elf->class;
}
