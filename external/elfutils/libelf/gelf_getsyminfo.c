
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <string.h>

#include "libelfP.h"


GElf_Syminfo *
gelf_getsyminfo (data, ndx, dst)
     Elf_Data *data;
     int ndx;
     GElf_Syminfo *dst;
{
  GElf_Syminfo *result = NULL;

  if (data == NULL)
    return NULL;

  if (unlikely (data->d_type != ELF_T_SYMINFO))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  /* The types for 32 and 64 bit are the same.  Lucky us.  */
  assert (sizeof (GElf_Syminfo) == sizeof (Elf32_Syminfo));
  assert (sizeof (GElf_Syminfo) == sizeof (Elf64_Syminfo));

  rwlock_rdlock (((Elf_Data_Scn *) data)->s->elf->lock);

  /* The data is already in the correct form.  Just make sure the
     index is OK.  */
  if (unlikely ((ndx + 1) * sizeof (GElf_Syminfo) > data->d_size))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      goto out;
    }

  *dst = ((GElf_Syminfo *) data->d_buf)[ndx];

  result = dst;

 out:
  rwlock_unlock (((Elf_Data_Scn *) data)->s->elf->lock);

  return result;
}
