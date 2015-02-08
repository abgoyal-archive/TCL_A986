
#include "util.h"
#include "unicode/unimatch.h"
#include "unicode/uniset.h"

// Define UChar constants using hex for EBCDIC compatibility

static const UChar BACKSLASH  = 0x005C; /*\*/
static const UChar UPPER_U    = 0x0055; /*U*/
static const UChar LOWER_U    = 0x0075; /*u*/
static const UChar APOSTROPHE = 0x0027; // '\''
static const UChar SPACE      = 0x0020; // ' '

// "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
static const UChar DIGITS[] = {
    48,49,50,51,52,53,54,55,56,57,
    65,66,67,68,69,70,71,72,73,74,
    75,76,77,78,79,80,81,82,83,84,
    85,86,87,88,89,90
};

U_NAMESPACE_BEGIN

UnicodeString& ICU_Utility::appendNumber(UnicodeString& result, int32_t n,
                                     int32_t radix, int32_t minDigits) {
    if (radix < 2 || radix > 36) {
        // Bogus radix
        return result.append((UChar)63/*?*/);
    }
    // Handle negatives
    if (n < 0) {
        n = -n;
        result.append((UChar)45/*-*/);
    }
    // First determine the number of digits
    int32_t nn = n;
    int32_t r = 1;
    while (nn >= radix) {
        nn /= radix;
        r *= radix;
        --minDigits;
    }
    // Now generate the digits
    while (--minDigits > 0) {
        result.append(DIGITS[0]);
    }
    while (r > 0) {
        int32_t digit = n / r;
        result.append(DIGITS[digit]);
        n -= digit * r;
        r /= radix;
    }
    return result;
}

UBool ICU_Utility::isUnprintable(UChar32 c) {
    return !(c >= 0x20 && c <= 0x7E);
}

UBool ICU_Utility::escapeUnprintable(UnicodeString& result, UChar32 c) {
    if (isUnprintable(c)) {
        result.append(BACKSLASH);
        if (c & ~0xFFFF) {
            result.append(UPPER_U);
            result.append(DIGITS[0xF&(c>>28)]);
            result.append(DIGITS[0xF&(c>>24)]);
            result.append(DIGITS[0xF&(c>>20)]);
            result.append(DIGITS[0xF&(c>>16)]);
        } else {
            result.append(LOWER_U);
        }
        result.append(DIGITS[0xF&(c>>12)]);
        result.append(DIGITS[0xF&(c>>8)]);
        result.append(DIGITS[0xF&(c>>4)]);
        result.append(DIGITS[0xF&c]);
        return TRUE;
    }
    return FALSE;
}

// FOR FUTURE USE.  DISABLE FOR NOW for coverage reasons.

int32_t ICU_Utility::skipWhitespace(const UnicodeString& str, int32_t& pos,
                                    UBool advance) {
    int32_t p = pos;
    while (p < str.length()) {
        UChar32 c = str.char32At(p);
        if (!uprv_isRuleWhiteSpace(c)) {
            break;
        }
        p += UTF_CHAR_LENGTH(c);
    }
    if (advance) {
        pos = p;
    }
    return p;
}

//?FOR FUTURE USE.  DISABLE FOR NOW for coverage reasons.
//?int32_t ICU_Utility::skipWhitespace(const Replaceable& text,
//?                                    int32_t pos, int32_t stop) {
//?    UChar32 c;
//?    UBool isForward = (stop >= pos);
//?
//?    if (!isForward) {
//?        --pos; // pos is a limit, so back up by one
//?    }
//?    
//?    while (pos != stop &&
//?           uprv_isRuleWhiteSpace(c = text.char32At(pos))) {
//?        if (isForward) {
//?            pos += UTF_CHAR_LENGTH(c);
//?        } else {
//?            pos -= UTF_CHAR_LENGTH(c);
//?        }
//?    }
//?
//?    if (!isForward) {
//?        ++pos; // make pos back into a limit
//?    }
//?
//?    return pos;
//?}

UBool ICU_Utility::parseChar(const UnicodeString& id, int32_t& pos, UChar ch) {
    int32_t start = pos;
    skipWhitespace(id, pos, TRUE);
    if (pos == id.length() ||
        id.charAt(pos) != ch) {
        pos = start;
        return FALSE;
    }
    ++pos;
    return TRUE;
}

int32_t ICU_Utility::parsePattern(const UnicodeString& pat,
                                  const Replaceable& text,
                                  int32_t index,
                                  int32_t limit) {
    int32_t ipat = 0;

    // empty pattern matches immediately
    if (ipat == pat.length()) {
        return index;
    }

    UChar32 cpat = pat.char32At(ipat);

    while (index < limit) {
        UChar32 c = text.char32At(index);

        // parse \s*
        if (cpat == 126 /*~*/) {
            if (uprv_isRuleWhiteSpace(c)) {
                index += UTF_CHAR_LENGTH(c);
                continue;
            } else {
                if (++ipat == pat.length()) {
                    return index; // success; c unparsed
                }
                // fall thru; process c again with next cpat
            }
        }

        // parse literal
        else if (c == cpat) {
            index += UTF_CHAR_LENGTH(c);
            ipat += UTF_CHAR_LENGTH(cpat);
            if (ipat == pat.length()) {
                return index; // success; c parsed
            }
            // fall thru; get next cpat
        }

        // match failure of literal
        else {
            return -1;
        }

        cpat = pat.char32At(ipat);
    }

    return -1; // text ended before end of pat
}

void ICU_Utility::appendToRule(UnicodeString& rule,
                               UChar32 c,
                               UBool isLiteral,
                               UBool escapeUnprintable,
                               UnicodeString& quoteBuf) {
    // If we are escaping unprintables, then escape them outside
    // quotes.  \u and \U are not recognized within quotes.  The same
    // logic applies to literals, but literals are never escaped.
    if (isLiteral ||
        (escapeUnprintable && ICU_Utility::isUnprintable(c))) {
        if (quoteBuf.length() > 0) {
            // We prefer backslash APOSTROPHE to double APOSTROPHE
            // (more readable, less similar to ") so if there are
            // double APOSTROPHEs at the ends, we pull them outside
            // of the quote.

            // If the first thing in the quoteBuf is APOSTROPHE
            // (doubled) then pull it out.
            while (quoteBuf.length() >= 2 &&
                   quoteBuf.charAt(0) == APOSTROPHE &&
                   quoteBuf.charAt(1) == APOSTROPHE) {
                rule.append(BACKSLASH).append(APOSTROPHE);
                quoteBuf.remove(0, 2);
            }
            // If the last thing in the quoteBuf is APOSTROPHE
            // (doubled) then remove and count it and add it after.
            int32_t trailingCount = 0;
            while (quoteBuf.length() >= 2 &&
                   quoteBuf.charAt(quoteBuf.length()-2) == APOSTROPHE &&
                   quoteBuf.charAt(quoteBuf.length()-1) == APOSTROPHE) {
                quoteBuf.truncate(quoteBuf.length()-2);
                ++trailingCount;
            }
            if (quoteBuf.length() > 0) {
                rule.append(APOSTROPHE);
                rule.append(quoteBuf);
                rule.append(APOSTROPHE);
                quoteBuf.truncate(0);
            }
            while (trailingCount-- > 0) {
                rule.append(BACKSLASH).append(APOSTROPHE);
            }
        }
        if (c != (UChar32)-1) {
            /* Since spaces are ignored during parsing, they are
             * emitted only for readability.  We emit one here
             * only if there isn't already one at the end of the
             * rule.
             */
            if (c == SPACE) {
                int32_t len = rule.length();
                if (len > 0 && rule.charAt(len-1) != c) {
                    rule.append(c);
                }
            } else if (!escapeUnprintable || !ICU_Utility::escapeUnprintable(rule, c)) {
                rule.append(c);
            }
        }
    }

    // Escape ' and '\' and don't begin a quote just for them
    else if (quoteBuf.length() == 0 &&
             (c == APOSTROPHE || c == BACKSLASH)) {
        rule.append(BACKSLASH);
        rule.append(c);
    }

    // Specials (printable ascii that isn't [0-9a-zA-Z]) and
    // whitespace need quoting.  Also append stuff to quotes if we are
    // building up a quoted substring already.
    else if (quoteBuf.length() > 0 ||
             (c >= 0x0021 && c <= 0x007E &&
              !((c >= 0x0030/*'0'*/ && c <= 0x0039/*'9'*/) ||
                (c >= 0x0041/*'A'*/ && c <= 0x005A/*'Z'*/) ||
                (c >= 0x0061/*'a'*/ && c <= 0x007A/*'z'*/))) ||
             uprv_isRuleWhiteSpace(c)) {
        quoteBuf.append(c);
        // Double ' within a quote
        if (c == APOSTROPHE) {
            quoteBuf.append(c);
        }
    }
    
    // Otherwise just append
    else {
        rule.append(c);
    }
}

void ICU_Utility::appendToRule(UnicodeString& rule,
                               const UnicodeString& text,
                               UBool isLiteral,
                               UBool escapeUnprintable,
                               UnicodeString& quoteBuf) {
    for (int32_t i=0; i<text.length(); ++i) {
        appendToRule(rule, text[i], isLiteral, escapeUnprintable, quoteBuf);
    }
}

void ICU_Utility::appendToRule(UnicodeString& rule,
                               const UnicodeMatcher* matcher,
                               UBool escapeUnprintable,
                               UnicodeString& quoteBuf) {
    if (matcher != NULL) {
        UnicodeString pat;
        appendToRule(rule, matcher->toPattern(pat, escapeUnprintable),
                     TRUE, escapeUnprintable, quoteBuf);
    }
}

U_NAMESPACE_END

U_CAPI UBool U_EXPORT2
uprv_isRuleWhiteSpace(UChar32 c) {
    /* "white space" in the sense of ICU rule parsers
       This is a FIXED LIST that is NOT DEPENDENT ON UNICODE PROPERTIES.
       See UAX #31 Identifier and Pattern Syntax: http://www.unicode.org/reports/tr31/
       U+0009..U+000D, U+0020, U+0085, U+200E..U+200F, and U+2028..U+2029
       Equivalent to test for Pattern_White_Space Unicode property.
    */
    return (c >= 0x0009 && c <= 0x2029 &&
            (c <= 0x000D || c == 0x0020 || c == 0x0085 ||
             c == 0x200E || c == 0x200F || c >= 0x2028));
}

U_CAPI U_NAMESPACE_QUALIFIER UnicodeSet* U_EXPORT2
uprv_openRuleWhiteSpaceSet(UErrorCode* ec) {
    if(U_FAILURE(*ec)) {
        return NULL;
    }
    // create a set with the Pattern_White_Space characters,
    // without a pattern for fewer code dependencies
    U_NAMESPACE_QUALIFIER UnicodeSet *set=new U_NAMESPACE_QUALIFIER UnicodeSet(9, 0xd);
    // Check for new failure.
    if (set == NULL) {
        *ec = U_MEMORY_ALLOCATION_ERROR;
        return NULL;
    }
    set->UnicodeSet::add(0x20).add(0x85).add(0x200e, 0x200f).add(0x2028, 0x2029);
    return set;
}

//eof
