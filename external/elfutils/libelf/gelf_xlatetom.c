
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf_Data *
gelf_xlatetom (elf, dest, src, encode)
     Elf *elf;
     Elf_Data *dest;
     const Elf_Data * src;
     unsigned int encode;
{
  if (elf == NULL)
    return NULL;

  return (elf->class == ELFCLASS32
	  ? INTUSE(elf32_xlatetom) (dest, src, encode)
	  : INTUSE(elf64_xlatetom) (dest, src, encode));
}
