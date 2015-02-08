
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <stdlib.h>

#include "libelfP.h"


int
gelf_update_versym (data, ndx, src)
     Elf_Data *data;
     int ndx;
     GElf_Versym *src;
{
  Elf_Data_Scn *data_scn = (Elf_Data_Scn *) data;

  if (data == NULL)
    return 0;

  /* The types for 32 and 64 bit are the same.  Lucky us.  */
  assert (sizeof (GElf_Versym) == sizeof (Elf32_Versym));
  assert (sizeof (GElf_Versym) == sizeof (Elf64_Versym));

  /* Check whether we have to resize the data buffer.  */
  if (unlikely (ndx < 0)
      || unlikely ((ndx + 1) * sizeof (GElf_Versym) > data_scn->d.d_size))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      return 0;
    }

  if (unlikely (data_scn->d.d_type != ELF_T_HALF))
    {
      /* The type of the data better should match.  */
      __libelf_seterrno (ELF_E_DATA_MISMATCH);
      return 0;
    }

  rwlock_wrlock (data_scn->s->elf->lock);

  ((GElf_Versym *) data_scn->d.d_buf)[ndx] = *src;

  /* Mark the section as modified.  */
  data_scn->s->flags |= ELF_F_DIRTY;

  rwlock_unlock (data_scn->s->elf->lock);

  return 1;
}
