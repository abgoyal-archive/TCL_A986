
#ifndef INTLTRANSLIT_H
#define INTLTRANSLIT_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "intltest.h"


class IntlTestTransliterator: public IntlTest {
public:
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
