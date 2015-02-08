#ifndef _NORMTST
#define _NORMTST

#include "cintltst.h"
    
    void TestDecomp(void);
    void TestCompatDecomp(void);
    void TestCanonDecompCompose(void);
    void TestCompatDecompCompose(void);
    void TestNull(void);
    void TestQuickCheck(void);
    void TestCheckFCD(void);

    /*internal functions*/
    
    static void assertEqual(const UChar* result,  const char* expected, int32_t index);


    


#endif
