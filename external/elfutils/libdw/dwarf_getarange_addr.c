
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <libdwP.h>


Dwarf_Arange *
dwarf_getarange_addr (aranges, addr)
     Dwarf_Aranges *aranges;
     Dwarf_Addr addr;
{
  if (aranges == NULL)
    return NULL;

  for (size_t cnt = 0; cnt < aranges->naranges; ++cnt)
    if (aranges->info[cnt].addr <= addr
        && addr < aranges->info[cnt].addr + aranges->info[cnt].length)
      return &aranges->info[cnt];

  __libdw_seterrno (DWARF_E_NO_MATCH);
  return NULL;
}
