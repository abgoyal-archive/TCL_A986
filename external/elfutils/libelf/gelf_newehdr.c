
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <gelf.h>
#include <stdlib.h>

#include "libelfP.h"


unsigned long int
gelf_newehdr (elf, class)
     Elf *elf;
     int class;
{
  return (class == ELFCLASS32
	  ? (unsigned long int) INTUSE(elf32_newehdr) (elf)
	  : (unsigned long int) INTUSE(elf64_newehdr) (elf));
}
