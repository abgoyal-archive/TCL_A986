
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <stdlib.h>

#include "libelfP.h"


unsigned long int
gelf_newphdr (elf, phnum)
     Elf *elf;
     size_t phnum;
{
  return (elf->class == ELFCLASS32
	  ? (unsigned long int) INTUSE(elf32_newphdr) (elf, phnum)
	  : (unsigned long int) INTUSE(elf64_newphdr) (elf, phnum));
}
