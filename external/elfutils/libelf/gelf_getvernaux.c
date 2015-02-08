
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <string.h>

#include "libelfP.h"


GElf_Vernaux *
gelf_getvernaux (data, offset, dst)
     Elf_Data *data;
     int offset;
     GElf_Vernaux *dst;
{
  GElf_Vernaux *result;

  if (data == NULL)
    return NULL;

  if (unlikely (data->d_type != ELF_T_VNEED))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  /* It's easy to handle this type.  It has the same size for 32 and
     64 bit objects.  And fortunately the `ElfXXX_Vernaux' records
     also have the same size.  */
  assert (sizeof (GElf_Vernaux) == sizeof (Elf32_Verneed));
  assert (sizeof (GElf_Vernaux) == sizeof (Elf64_Verneed));
  assert (sizeof (GElf_Vernaux) == sizeof (Elf32_Vernaux));
  assert (sizeof (GElf_Vernaux) == sizeof (Elf64_Vernaux));

  rwlock_rdlock (((Elf_Data_Scn *) data)->s->elf->lock);

  /* The data is already in the correct form.  Just make sure the
     index is OK.  */
  if (unlikely (offset < 0)
      || unlikely (offset + sizeof (GElf_Vernaux) > data->d_size)
      || unlikely (offset % sizeof (GElf_Vernaux) != 0))
    {
      __libelf_seterrno (ELF_E_OFFSET_RANGE);
      result = NULL;
    }
  else
    result = (GElf_Vernaux *) memcpy (dst, (char *) data->d_buf + offset,
				      sizeof (GElf_Verneed));

  rwlock_unlock (((Elf_Data_Scn *) data)->s->elf->lock);

  return result;
}
