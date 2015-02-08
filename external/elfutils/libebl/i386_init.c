
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_i386.h>


int
i386_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "Intel 80386";
  eh->reloc_type_name = i386_reloc_type_name;
  eh->reloc_type_check = i386_reloc_type_check;
  eh->reloc_valid_use = i386_reloc_valid_use;
  eh->gotpc_reloc_check = i386_gotpc_reloc_check;
  eh->core_note = i386_core_note;
  eh->destr = i386_destr;

  return 0;
}
