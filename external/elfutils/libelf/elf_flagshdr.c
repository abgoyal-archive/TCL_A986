
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


unsigned int
elf_flagshdr (scn, cmd, flags)
     Elf_Scn *scn;
     Elf_Cmd cmd;
     unsigned int flags;
{
  unsigned int result;

  if (scn == NULL)
    return 0;

  if (unlikely (scn->elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return 0;
    }

  if (likely (cmd == ELF_C_SET))
    result = (scn->shdr_flags |= (flags & ELF_F_DIRTY));
  else if (likely (cmd == ELF_C_CLR))
    result = (scn->shdr_flags &= ~(flags & ELF_F_DIRTY));
  else
    {
      __libelf_seterrno (ELF_E_INVALID_COMMAND);
      return 0;
    }

  return result;
}
