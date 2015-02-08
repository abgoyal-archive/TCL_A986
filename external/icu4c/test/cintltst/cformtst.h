/* FormatTest is a medium top level test for everything in the  C FORMAT API */

#ifndef _CFORMATTST
#define _CFORMATTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "cintltst.h"
#include "unicode/udat.h"


/* Internal fucntion used by all the test format files */
UChar* myDateFormat(UDateFormat *dat, UDate d); 

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
