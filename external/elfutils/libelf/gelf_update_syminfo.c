
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <stdlib.h>

#include "libelfP.h"


int
gelf_update_syminfo (data, ndx, src)
     Elf_Data *data;
     int ndx;
     GElf_Syminfo *src;
{
  Elf_Data_Scn *data_scn = (Elf_Data_Scn *) data;
  Elf_Scn *scn;
  int result = 0;

  if (data == NULL)
    return 0;

  if (unlikely (ndx < 0))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      return 0;
    }

  if (unlikely (data_scn->d.d_type != ELF_T_SYMINFO))
    {
      /* The type of the data better should match.  */
      __libelf_seterrno (ELF_E_DATA_MISMATCH);
      return 0;
    }

  /* The types for 32 and 64 bit are the same.  Lucky us.  */
  assert (sizeof (GElf_Syminfo) == sizeof (Elf32_Syminfo));
  assert (sizeof (GElf_Syminfo) == sizeof (Elf64_Syminfo));

  scn = data_scn->s;
  rwlock_wrlock (scn->elf->lock);

  /* Check whether we have to resize the data buffer.  */
  if (unlikely ((ndx + 1) * sizeof (GElf_Syminfo) > data_scn->d.d_size))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      goto out;
    }

  ((GElf_Syminfo *) data_scn->d.d_buf)[ndx] = *src;

  result = 1;

  /* Mark the section as modified.  */
  scn->flags |= ELF_F_DIRTY;

 out:
  rwlock_unlock (scn->elf->lock);

  return result;
}
