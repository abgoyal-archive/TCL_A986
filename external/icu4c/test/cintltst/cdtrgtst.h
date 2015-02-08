/* REGRESSION TEST FOR DATE FORMAT */
#ifndef _CDTFRRGSTST
#define _CDTFRRGSTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "cintltst.h"

    /**
     * DateFormat Regression tests
     **/

    void Test4029195(void); 
    void Test4056591(void); 
    void Test4059917(void);
    void Test4060212(void); 
    void Test4061287(void); 
    void Test4073003(void); 
    void Test4162071(void); 
    void Test714(void);
    void Test_GEec(void);

    /**
     * test subroutine
     **/
    void aux917(UDateFormat *fmt, UChar* str );

    /**
     * test subroutine used by the testing functions
     **/
    UChar* myFormatit(UDateFormat* datdef, UDate d1);

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
