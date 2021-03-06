
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include <stddef.h>

#include "libelfP.h"


unsigned int
elf_flagelf (elf, cmd, flags)
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
    result = (elf->flags
	      |= (flags & (ELF_F_DIRTY | ELF_F_LAYOUT | ELF_F_PERMISSIVE)));
  else if (likely (cmd == ELF_C_CLR))
    result = (elf->flags
	      &= ~(flags & (ELF_F_DIRTY | ELF_F_LAYOUT | ELF_F_PERMISSIVE)));
  else
    {
      __libelf_seterrno (ELF_E_INVALID_COMMAND);
      return 0;
    }

  return result;
}
