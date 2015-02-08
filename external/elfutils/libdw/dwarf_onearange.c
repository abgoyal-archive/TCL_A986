
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libdwP.h"


Dwarf_Arange *
dwarf_onearange (Dwarf_Aranges *aranges, size_t idx)
{
  if (aranges == NULL)
    return NULL;

  if (idx >= aranges->naranges)
    {
      __libdw_seterrno (DWARF_E_INVALID_ARANGE_IDX);
      return NULL;
    }

  return &aranges->info[idx];
}
