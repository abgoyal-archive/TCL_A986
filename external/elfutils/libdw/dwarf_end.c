
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <search.h>
#include <stdlib.h>

#include "libdwP.h"



static void
noop_free (void *arg)
{
}


static void
cu_free (void *arg)
{
  struct Dwarf_CU *p = (struct Dwarf_CU *) arg;

  Dwarf_Abbrev_Hash_free (&p->abbrev_hash);

  tdestroy (p->locs, noop_free);
}


int
dwarf_end (dwarf)
     Dwarf *dwarf;
{
  if (dwarf != NULL)
    {
      /* The search tree for the CUs.  NB: the CU data itself is
	 allocated separately, but the abbreviation hash tables need
	 to be handled.  */
      tdestroy (dwarf->cu_tree, cu_free);

      struct libdw_memblock *memp = dwarf->mem_tail;
      /* The first block is allocated together with the Dwarf object.  */
      while (memp->prev != NULL)
	{
	  struct libdw_memblock *prevp = memp->prev;
	  free (memp);
	  memp = prevp;
	}

      /* Free the pubnames helper structure.  */
      free (dwarf->pubnames_sets);

      /* Free the ELF descriptor if necessary.  */
      if (dwarf->free_elf)
	elf_end (dwarf->elf);

      /* Free the context descriptor.  */
      free (dwarf);
    }

  return 0;
}
