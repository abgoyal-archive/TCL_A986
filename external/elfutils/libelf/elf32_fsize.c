
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelf.h>
#include "libelfP.h"

#ifndef LIBELFBITS
# define LIBELFBITS 32
#endif


size_t
elfw2(LIBELFBITS, fsize) (type, count, version)
     Elf_Type type;
     size_t count;
     unsigned int version;
{
  /* We do not have differences between file and memory sizes.  Better
     not since otherwise `mmap' would not work.  */
  if (unlikely (version == EV_NONE) || unlikely (version >= EV_NUM))
    {
      __libelf_seterrno (ELF_E_UNKNOWN_VERSION);
      return 0;
    }

  if (unlikely (type >= ELF_T_NUM))
    {
      __libelf_seterrno (ELF_E_UNKNOWN_TYPE);
      return 0;
    }

#if EV_NUM != 2
  return (count
	  * __libelf_type_sizes[version - 1][ELFW(ELFCLASS,LIBELFBITS) - 1][type]);
#else
  return (count
	  * __libelf_type_sizes[0][ELFW(ELFCLASS,LIBELFBITS) - 1][type]);
#endif
}
#define local_strong_alias(n1, n2) strong_alias (n1, n2)
//local_strong_alias (elfw2(LIBELFBITS, fsize), __elfw2(LIBELFBITS, msize))
