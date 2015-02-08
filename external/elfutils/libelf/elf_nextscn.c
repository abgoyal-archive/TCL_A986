
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


Elf_Scn *
elf_nextscn (elf, scn)
     Elf *elf;
     Elf_Scn *scn;
{
  Elf_Scn *result = NULL;

  if (elf == NULL)
    return NULL;

  rwlock_rdlock (elf->lock);

  if (scn == NULL)
    {
      /* If no section handle is given return the first (not 0th) section.  */
      if (elf->class == ELFCLASS32
	   || (offsetof (Elf, state.elf32.scns)
	       == offsetof (Elf, state.elf64.scns)))
	{
	  if (elf->state.elf32.scns.cnt > 1)
	    result = &elf->state.elf32.scns.data[1];
	}
      else
	{
	  if (elf->state.elf64.scns.cnt > 1)
	    result = &elf->state.elf64.scns.data[1];
	}
    }
  else
    {
      Elf_ScnList *list = scn->list;

      if (scn + 1 < &list->data[list->cnt])
	result = scn + 1;
      else if (scn + 1 == &list->data[list->max]
	       && (list = list->next) != NULL)
	{
	  /* If there is another element in the section list it must
	     have at least one entry.  */
	  assert (list->cnt > 0);
	  result = &list->data[0];
	}
    }

  rwlock_unlock (elf->lock);

  return result;
}
INTDEF(elf_nextscn)
