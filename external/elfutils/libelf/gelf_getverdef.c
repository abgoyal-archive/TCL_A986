
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <string.h>

#include "libelfP.h"


GElf_Verdef *
gelf_getverdef (data, offset, dst)
     Elf_Data *data;
     int offset;
     GElf_Verdef *dst;
{
  GElf_Verdef *result;

  if (data == NULL)
    return NULL;

  if (unlikely (data->d_type != ELF_T_VDEF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  /* It's easy to handle this type.  It has the same size for 32 and
     64 bit objects.  */
  assert (sizeof (GElf_Verdef) == sizeof (Elf32_Verdef));
  assert (sizeof (GElf_Verdef) == sizeof (Elf64_Verdef));

  rwlock_rdlock (((Elf_Data_Scn *) data)->s->elf->lock);

  /* The data is already in the correct form.  Just make sure the
     index is OK.  */
  if (unlikely (offset < 0)
      || unlikely (offset + sizeof (GElf_Verdef) > data->d_size)
      || unlikely (offset % __alignof__ (GElf_Verdef) != 0))
    {
      __libelf_seterrno (ELF_E_OFFSET_RANGE);
      result = NULL;
    }
  else
    result = (GElf_Verdef *) memcpy (dst, (char *) data->d_buf + offset,
				     sizeof (GElf_Verdef));

  rwlock_unlock (((Elf_Data_Scn *) data)->s->elf->lock);

  return result;
}
