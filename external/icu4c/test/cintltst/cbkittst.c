
#include "unicode/utypes.h"

#if !UCONFIG_NO_BREAK_ITERATION

#include "cintltst.h"

void addBrkIterAPITest(TestNode**);

void addBreakIter(TestNode** root);

void addBreakIter(TestNode** root)
{
    addBrkIterAPITest(root);
}

#endif /* #if !UCONFIG_NO_BREAK_ITERATION */
