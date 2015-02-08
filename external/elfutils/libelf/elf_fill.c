
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>

#include "libelfP.h"

extern int __libelf_fill_byte = 0;

void
elf_fill (fill)
     int fill;
{
  __libelf_fill_byte = fill;
}
