
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libelfP.h>

/* Get the implementation.  */
#include <dl-hash.h>

unsigned long int
elf_hash (string)
     const char *string;
{
  return _dl_elf_hash (string);
}
INTDEF(elf_hash)
