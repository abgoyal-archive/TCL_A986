
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


unsigned int
elf_flagehdr (elf, cmd, flags)
     Elf *elf;
     Elf_Cmd cmd;
     unsigned int flags;
{
  unsigned int result;

  if (elf == NULL)
    return 0;

  if (unlikely (elf->kind != ELF_K_ELF))
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return 0;
    }

  if (likely (cmd == ELF_C_SET))
    result = (elf->state.elf.ehdr_flags |= (flags & ELF_F_DIRTY));
  else if (cmd == ELF_C_CLR)
    result = (elf->state.elf.ehdr_flags &= ~(flags & ELF_F_DIRTY));
  else
    {
      __libelf_seterrno (ELF_E_INVALID_COMMAND);
      return 0;
    }

  return result;
}
