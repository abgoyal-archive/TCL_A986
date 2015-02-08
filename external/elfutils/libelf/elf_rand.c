
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


size_t
elf_rand (elf, offset)
     Elf *elf;
     size_t offset;
{
  /* Be gratious, the specs demand it.  */
  if (elf == NULL || elf->kind != ELF_K_AR)
    return 0;

  /* Save the old offset and set the offset.  */
  elf->state.ar.offset = elf->start_offset + offset;

  /* Get the next archive header.  */
  if (__libelf_next_arhdr (elf) != 0)
    {
      /* Mark the archive header as unusable.  */
      elf->state.ar.elf_ar_hdr.ar_name = NULL;
      return 0;
    }

  return offset;
}
