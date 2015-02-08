
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_sh.h>


int
sh_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "Hitachi SH";
  eh->reloc_type_name = sh_reloc_type_name;
  eh->destr = sh_destr;

  return 0;
}
