
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


off_t
elf_getbase (elf)
     Elf *elf;
{
  return elf == NULL ? (off_t) -1 : elf->start_offset;
}
