
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <stddef.h>

#include "libelfP.h"


long int
gelf_checksum (elf)
     Elf *elf;
{
  if (elf == NULL)
    return -1l;

  return (elf->class == ELFCLASS32
	  ? INTUSE(elf32_checksum) (elf) : INTUSE(elf64_checksum) (elf));
}
