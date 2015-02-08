
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>

#include "libasmP.h"


AsmScn_t *
asm_newscn_ingrp (ctx, scnname, type, flags, grp)
     AsmCtx_t *ctx;
     const char *scnname;
     GElf_Word type;
     GElf_Xword flags;
     AsmScnGrp_t *grp;
{
  AsmScn_t *result = __asm_newscn_internal (ctx, scnname, type, flags);

  if (likely (result != NULL))
    {
      /* We managed to create a section group.  Add it to the section
	 group.  */
      if (grp->nmembers == 0)
	{
	  assert (grp->members == NULL);
	  grp->members = result->data.main.next_in_group = result;
	}
      else
	{
	  result->data.main.next_in_group
	    = grp->members->data.main.next_in_group;
	  grp->members = grp->members->data.main.next_in_group = result;
	}

      ++grp->nmembers;

      /* Set the SHF_GROUP flag.  */
      if (likely (! ctx->textp))
	{
	  GElf_Shdr shdr_mem;
	  GElf_Shdr *shdr = gelf_getshdr (result->data.main.scn, &shdr_mem);

	  assert (shdr != NULL);
	  shdr->sh_flags |= SHF_GROUP;

	  (void) gelf_update_shdr (result->data.main.scn, shdr);
	}
    }

  return result;
}
