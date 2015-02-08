
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_alpha.h>


int
alpha_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "Alpha";
  eh->reloc_type_name = alpha_reloc_type_name;
  eh->reloc_type_check = alpha_reloc_type_check;
  eh->destr = alpha_destr;

  return 0;
}
