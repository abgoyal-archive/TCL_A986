
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


unsigned int
elf_flagdata (data, cmd, flags)
     Elf_Data *data;
     Elf_Cmd cmd;
     unsigned int flags;
{
  Elf_Data_Scn *data_scn;
  unsigned int result;

  if (data == NULL)
    return 0;

  data_scn = (Elf_Data_Scn *) data;

  if (data_scn == NULL || unlikely (data_scn->s->elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return 0;
    }

  if (likely (cmd == ELF_C_SET))
    result = (data_scn->s->flags |= (flags & ELF_F_DIRTY));
  else if (likely (cmd == ELF_C_CLR))
    result = (data_scn->s->flags &= ~(flags & ELF_F_DIRTY));
  else
    {
      __libelf_seterrno (ELF_E_INVALID_COMMAND);
      return 0;
    }

  return result;
}
