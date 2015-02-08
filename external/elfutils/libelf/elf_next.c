
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf_Cmd
elf_next (elf)
     Elf *elf;
{
  Elf *parent;

  /* Be gratious, the specs demand it.  */
  if (elf == NULL || elf->parent == NULL)
    return ELF_C_NULL;

  /* We can be sure the parent is an archive.  */
  parent = elf->parent;
  assert (parent->kind == ELF_K_AR);

  /* Now advance the offset.  */
  parent->state.ar.offset += (sizeof (struct ar_hdr)
			      + ((parent->state.ar.elf_ar_hdr.ar_size + 1)
				 & ~1l));

  /* Get the next archive header.  */
  if (__libelf_next_arhdr (parent) != 0)
    return ELF_C_NULL;

  return elf->cmd;
}
