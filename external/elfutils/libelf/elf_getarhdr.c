
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf_Arhdr *
elf_getarhdr (elf)
     Elf *elf;
{
  Elf *parent = elf->parent;

  /* Calling this function is not ok for any file type but archives.  */
  if (parent == NULL)
    {
      __libelf_seterrno (ELF_E_INVALID_OP);
      return NULL;
    }

  /* Make sure we have read the archive header.  */
  if (parent->state.ar.elf_ar_hdr.ar_name == NULL
      && __libelf_next_arhdr (parent) != 0)
    /* Something went wrong.  Maybe there is no member left.  */
    return NULL;


  /* We can be sure the parent is an archive.  */
  assert (parent->kind == ELF_K_AR);

  return &parent->state.ar.elf_ar_hdr;
}
