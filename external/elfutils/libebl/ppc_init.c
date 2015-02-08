
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_ppc.h>


int
ppc_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "PowerPC";
  eh->reloc_type_name = ppc_reloc_type_name;
  eh->reloc_type_check = ppc_reloc_type_check;
  eh->reloc_valid_use = ppc_reloc_valid_use;
  eh->destr = ppc_destr;

  return 0;
}
