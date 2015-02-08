
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libebl_arm.h>


int
arm_init (elf, machine, eh, ehlen)
     Elf *elf;
     GElf_Half machine;
     Ebl *eh;
     size_t ehlen;
{
  /* Check whether the Elf_BH object has a sufficent size.  */
  if (ehlen < sizeof (Ebl))
    return 1;

  /* We handle it.  */
  eh->name = "Arm";
  eh->reloc_type_name = arm_reloc_type_name;
  eh->reloc_type_check = arm_reloc_type_check;
  eh->destr = arm_destr;

  return 0;
}
