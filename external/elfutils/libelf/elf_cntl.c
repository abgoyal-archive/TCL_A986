
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <unistd.h>

#include "libelfP.h"


int
elf_cntl (elf, cmd)
     Elf *elf;
     Elf_Cmd cmd;
{
  int result = 0;

  if (elf == NULL)
    return -1;

  if (elf->fildes == -1)
    {
      __libelf_seterrno (ELF_E_INVALID_HANDLE);
      return -1;
    }

  rwlock_wrlock (elf->lock);

  switch (cmd)
    {
    case ELF_C_FDREAD:
      /* If not all of the file is in the memory read it now.  */
      if (elf->map_address == NULL && __libelf_readall (elf) == NULL)
	{
	  /* We were not able to read everything.  */
	  result = -1;
	  break;
	}
      /* FALLTHROUGH */

    case ELF_C_FDDONE:
      /* Mark the file descriptor as not usable.  */
      elf->fildes = -1;
      break;

    default:
      __libelf_seterrno (ELF_E_INVALID_CMD);
      result = -1;
      break;
    }

  rwlock_unlock (elf->lock);

  return result;
}
