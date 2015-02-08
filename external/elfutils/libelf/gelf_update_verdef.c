
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <string.h>

#include "libelfP.h"


int
gelf_update_verdef (data, offset, src)
     Elf_Data *data;
     int offset;
     GElf_Verdef *src;
{
  Elf_Data_Scn *data_scn = (Elf_Data_Scn *) data;

  if (data == NULL)
    return 0;

  /* The types for 32 and 64 bit are the same.  Lucky us.  */
  assert (sizeof (GElf_Verdef) == sizeof (Elf32_Verdef));
  assert (sizeof (GElf_Verdef) == sizeof (Elf64_Verdef));

  /* Check whether we have to resize the data buffer.  */
  if (unlikely (offset < 0)
      || unlikely ((offset + sizeof (GElf_Verdef)) > data_scn->d.d_size))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      return 0;
    }

  if (unlikely (data_scn->d.d_type != ELF_T_VDEF))
    {
      /* The type of the data better should match.  */
      __libelf_seterrno (ELF_E_DATA_MISMATCH);
      return 0;
    }

  rwlock_wrlock (data_scn->s->elf->lock);

  memcpy ((char *) data_scn->d.d_buf + offset, src, sizeof (GElf_Verdef));

  /* Mark the section as modified.  */
  data_scn->s->flags |= ELF_F_DIRTY;

  rwlock_unlock (data_scn->s->elf->lock);

  return 1;
}
