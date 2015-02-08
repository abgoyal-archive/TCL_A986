
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_sparc.h>


int
sparc_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  if (machine == EM_SPARCV9)
    eh->name = "SPARC v9";
  else if (machine == EM_SPARC32PLUS)
    eh->name = "SPARC v8+";
  else
    eh->name = "SPARC";
  eh->reloc_type_name = sparc_reloc_type_name;
  eh->reloc_type_check = sparc_reloc_type_check;
  //eh->core_note = sparc_core_note;
  eh->destr = sparc_destr;

  return 0;
}
