
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_ia64.h>


int
ia64_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "Intel IA-64";
  eh->reloc_type_name = ia64_reloc_type_name;
  eh->reloc_type_check = ia64_reloc_type_check;
  eh->segment_type_name = ia64_segment_type_name;
  eh->dynamic_tag_name = ia64_dynamic_tag_name;
  eh->destr = ia64_destr;

  return 0;
}
