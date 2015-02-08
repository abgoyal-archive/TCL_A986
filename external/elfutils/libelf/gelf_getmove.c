
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <assert.h>
#include <gelf.h>
#include <stddef.h>

#include "libelfP.h"


GElf_Move *
gelf_getmove (data, ndx, dst)
     Elf_Data *data;
     int ndx;
     GElf_Move *dst;
{
  GElf_Move *result = NULL;
  Elf *elf;

  if (data == NULL)
    return NULL;

  if (unlikely (data->d_type != ELF_T_MOVE))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return NULL;
    }

  /* The types for 32 and 64 bit are the same.  Lucky us.  */
  assert (sizeof (GElf_Move) == sizeof (Elf32_Move));
  assert (sizeof (GElf_Move) == sizeof (Elf64_Move));

  /* The data is already in the correct form.  Just make sure the
     index is OK.  */
  if (unlikely ((ndx + 1) * sizeof (GElf_Move) > data->d_size))
    {
      __libelf_seterrno (ELF_E_INVALID_INDEX);
      goto out;
    }

  elf = ((Elf_Data_Scn *) data)->s->elf;
  rwlock_rdlock (elf->lock);

  *dst = ((GElf_Move *) data->d_buf)[ndx];

  rwlock_unlock (elf->lock);

  result = dst;

 out:
  return result;
}
