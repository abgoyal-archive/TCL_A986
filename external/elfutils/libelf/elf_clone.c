
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <stddef.h>
#include "libelfP.h"
#include "common.h"


Elf *
elf_clone (Elf *elf, Elf_Cmd cmd)
{
  Elf *retval = NULL;

  if (elf == NULL)
    /* Some earlier mistake.  */
    return NULL;

  /* Make sure the descriptor is not suddenly going away.  */
  rwlock_rdlock (elf->lock);

  if (cmd != ELF_C_EMPTY)
    // XXX TODO handle ELF_C_READ/WRITE etc
    goto out;

  retval = allocate_elf (elf->fildes, elf->map_address, elf->start_offset,
			 elf->maximum_size, elf->cmd, elf->parent, elf->kind,
			 elf->state.elf32.scns.max * sizeof (Elf_Scn));
  if (retval != NULL)
    {
      /* We have to write to the file in any case.  */
      retval->flags = ELF_F_DIRTY;

      /* Some more or less arbitrary value.  */
      retval->state.elf.scnincr = 10;

      /* We have allocated room for some sections.  */
      assert (offsetof (struct Elf, state.elf32.scns)
	      == offsetof (struct Elf, state.elf64.scns));
      retval->state.elf.scns_last = &retval->state.elf32.scns;
      retval->state.elf32.scns.max = elf->state.elf32.scns.max;

      retval->class = elf->class;
    }

  /* Release the lock.  */
 out:
  rwlock_unlock (elf->lock);

  return retval;
}
