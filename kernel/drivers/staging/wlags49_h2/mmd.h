

//   vim:tw=110:ts=4:
#ifndef MMD_H
#define MMD_H 1

#ifndef HCF_H
#include "hcf.h"	//just to get going with swig
#endif

EXTERN_C CFG_RANGE_SPEC_STRCT* mmd_check_comp( CFG_RANGES_STRCT *actp, CFG_SUP_RANGE_STRCT *supp );

#endif // MMD_H
