
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "libasmP.h"


int
asm_scngrp_newsignature (grp, signature)
     AsmScnGrp_t *grp;
     AsmSym_t *signature;
{
  if (grp == NULL || signature == NULL)
    return 1;

  grp->signature =  signature;

  return 0;
}
