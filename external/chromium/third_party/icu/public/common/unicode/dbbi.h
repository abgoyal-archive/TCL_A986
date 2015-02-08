
#ifndef DBBI_H
#define DBBI_H

#include "unicode/rbbi.h"

#if !UCONFIG_NO_BREAK_ITERATION

 
U_NAMESPACE_BEGIN

 
#ifndef U_HIDE_DEPRECATED_API

typedef RuleBasedBreakIterator DictionaryBasedBreakIterator;

#endif

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_BREAK_ITERATION */

#endif
