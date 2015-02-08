
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_ppc64.h>


int
ppc64_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "PowerPC 64-bit";
  eh->reloc_type_name = ppc64_reloc_type_name;
  eh->reloc_type_check = ppc64_reloc_type_check;
  eh->reloc_valid_use = ppc64_reloc_valid_use;
  eh->dynamic_tag_name = ppc64_dynamic_tag_name;
  eh->destr = ppc64_destr;

  return 0;
}
