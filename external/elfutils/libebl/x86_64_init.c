
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_x86_64.h>


int
x86_64_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "AMD x86-64";
  eh->reloc_type_name = x86_64_reloc_type_name;
  eh->reloc_type_check = x86_64_reloc_type_check;
  eh->reloc_valid_use = x86_64_reloc_valid_use;
  //eh->core_note = i386_core_note;
  eh->destr = x86_64_destr;

  return 0;
}
