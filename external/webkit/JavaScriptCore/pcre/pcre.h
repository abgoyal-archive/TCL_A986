

// FIXME: This file needs to be renamed to JSRegExp.h; it's no longer PCRE.

#ifndef JSRegExp_h
#define JSRegExp_h

#include <wtf/unicode/Unicode.h>

struct JSRegExp;

enum JSRegExpIgnoreCaseOption { JSRegExpDoNotIgnoreCase, JSRegExpIgnoreCase };
enum JSRegExpMultilineOption { JSRegExpSingleLine, JSRegExpMultiline };

/* jsRegExpExecute error codes */
const int JSRegExpErrorNoMatch = -1;
const int JSRegExpErrorHitLimit = -2;
const int JSRegExpErrorNoMemory = -3;
const int JSRegExpErrorInternal = -4;

JSRegExp* jsRegExpCompile(const UChar* pattern, int patternLength,
    JSRegExpIgnoreCaseOption, JSRegExpMultilineOption,
    unsigned* numSubpatterns, const char** errorMessage);

int jsRegExpExecute(const JSRegExp*,
    const UChar* subject, int subjectLength, int startOffset,
    int* offsetsVector, int offsetsVectorLength);

void jsRegExpFree(JSRegExp*);

#endif
