
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <inttypes.h>
#include <libeblP.h>


bool
ebl_dynamic_tag_check (ebl, tag)
     Ebl *ebl;
     int64_t tag;
{
  bool res = ebl != NULL ? ebl->dynamic_tag_check (tag) : false;

  if (!res
      && ((tag >= 0 && tag < DT_NUM)
	  || (tag >= DT_GNU_PRELINKED && tag <= DT_SYMINENT)
	  || (tag >= DT_GNU_CONFLICT && tag <= DT_SYMINFO)
	  || tag == DT_VERSYM
	  || (tag >= DT_RELACOUNT && tag <= DT_VERNEEDNUM)
	  || tag == DT_AUXILIARY
	  || tag == DT_FILTER))
    res = true;

  return res;
}
