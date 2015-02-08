
#ifndef _CG7COLLTST
#define _CG7COLLTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "cintltst.h"

#define MAX_TOKEN_LEN 16
#define  TESTLOCALES  12 
#define  FIXEDTESTSET 15 
#define  TOTALTESTSET  30 

    /* perform test for G7 locales */
    static void TestG7Locales(void);

    /* perform test with added rules " & Z < p, P" */
    static void TestDemo1(void);

    /* perorm test with added rules "& C < ch , cH, Ch, CH" */
    static void TestDemo2(void);

    /* perform test with added rules  */
    /* "& Question'-'mark ; '?' & Hash'-'mark ; '#' & Ampersand ; '&'" */
    static void TestDemo3(void);

    /* perform test with added rules  */
    /* " & aa ; a'-' & ee ; e'-' & ii ; i'-' & oo ; o'-' & uu ; u'-' " */
    static void TestDemo4(void);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
