
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "strmatch.h"
#include "rbt_data.h"
#include "util.h"
#include "unicode/uniset.h"

U_NAMESPACE_BEGIN

static const UChar EMPTY[] = { 0 }; // empty string: ""

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(StringMatcher)

StringMatcher::StringMatcher(const UnicodeString& theString,
                             int32_t start,
                             int32_t limit,
                             int32_t segmentNum,
                             const TransliterationRuleData& theData) :
    data(&theData),
    segmentNumber(segmentNum),
    matchStart(-1),
    matchLimit(-1)
{
    theString.extractBetween(start, limit, pattern);
}

StringMatcher::StringMatcher(const StringMatcher& o) :
    UnicodeFunctor(o),
    UnicodeMatcher(o),
    UnicodeReplacer(o),
    pattern(o.pattern),
    data(o.data),
    segmentNumber(o.segmentNumber),
    matchStart(o.matchStart),
    matchLimit(o.matchLimit)
{
}

StringMatcher::~StringMatcher() {
}

UnicodeFunctor* StringMatcher::clone() const {
    return new StringMatcher(*this);
}

UnicodeMatcher* StringMatcher::toMatcher() const {
    return (UnicodeMatcher*) this;
}

UnicodeReplacer* StringMatcher::toReplacer() const {
    return (UnicodeReplacer*) this;
}

UMatchDegree StringMatcher::matches(const Replaceable& text,
                                    int32_t& offset,
                                    int32_t limit,
                                    UBool incremental) {
    int32_t i;
    int32_t cursor = offset;
    if (limit < cursor) {
        // Match in the reverse direction
        for (i=pattern.length()-1; i>=0; --i) {
            UChar keyChar = pattern.charAt(i);
            UnicodeMatcher* subm = data->lookupMatcher(keyChar);
            if (subm == 0) {
                if (cursor > limit &&
                    keyChar == text.charAt(cursor)) {
                    --cursor;
                } else {
                    return U_MISMATCH;
                }
            } else {
                UMatchDegree m =
                    subm->matches(text, cursor, limit, incremental);
                if (m != U_MATCH) {
                    return m;
                }
            }
        }
        // Record the match position, but adjust for a normal
        // forward start, limit, and only if a prior match does not
        // exist -- we want the rightmost match.
        if (matchStart < 0) {
            matchStart = cursor+1;
            matchLimit = offset+1;
        }
    } else {
        for (i=0; i<pattern.length(); ++i) {
            if (incremental && cursor == limit) {
                // We've reached the context limit without a mismatch and
                // without completing our match.
                return U_PARTIAL_MATCH;
            }
            UChar keyChar = pattern.charAt(i);
            UnicodeMatcher* subm = data->lookupMatcher(keyChar);
            if (subm == 0) {
                // Don't need the cursor < limit check if
                // incremental is TRUE (because it's done above); do need
                // it otherwise.
                if (cursor < limit &&
                    keyChar == text.charAt(cursor)) {
                    ++cursor;
                } else {
                    return U_MISMATCH;
                }
            } else {
                UMatchDegree m =
                    subm->matches(text, cursor, limit, incremental);
                if (m != U_MATCH) {
                    return m;
                }
            }
        }
        // Record the match position
        matchStart = offset;
        matchLimit = cursor;
    }

    offset = cursor;
    return U_MATCH;
}

UnicodeString& StringMatcher::toPattern(UnicodeString& result,
                                        UBool escapeUnprintable) const
{
    result.truncate(0);
    UnicodeString str, quoteBuf;
    if (segmentNumber > 0) {
        result.append((UChar)40); /*(*/
    }
    for (int32_t i=0; i<pattern.length(); ++i) {
        UChar keyChar = pattern.charAt(i);
        const UnicodeMatcher* m = data->lookupMatcher(keyChar);
        if (m == 0) {
            ICU_Utility::appendToRule(result, keyChar, FALSE, escapeUnprintable, quoteBuf);
        } else {
            ICU_Utility::appendToRule(result, m->toPattern(str, escapeUnprintable),
                         TRUE, escapeUnprintable, quoteBuf);
        }
    }
    if (segmentNumber > 0) {
        result.append((UChar)41); /*)*/
    }
    // Flush quoteBuf out to result
    ICU_Utility::appendToRule(result, -1,
                              TRUE, escapeUnprintable, quoteBuf);
    return result;
}

UBool StringMatcher::matchesIndexValue(uint8_t v) const {
    if (pattern.length() == 0) {
        return TRUE;
    }
    UChar32 c = pattern.char32At(0);
    const UnicodeMatcher *m = data->lookupMatcher(c);
    return (m == 0) ? ((c & 0xFF) == v) : m->matchesIndexValue(v);
}

void StringMatcher::addMatchSetTo(UnicodeSet& toUnionTo) const {
    UChar32 ch;
    for (int32_t i=0; i<pattern.length(); i+=UTF_CHAR_LENGTH(ch)) {
        ch = pattern.char32At(i);
        const UnicodeMatcher* matcher = data->lookupMatcher(ch);
        if (matcher == NULL) {
            toUnionTo.add(ch);
        } else {
            matcher->addMatchSetTo(toUnionTo);
        }
    }
}

int32_t StringMatcher::replace(Replaceable& text,
                               int32_t start,
                               int32_t limit,
                               int32_t& /*cursor*/) {
    
    int32_t outLen = 0;
    
    // Copy segment with out-of-band data
    int32_t dest = limit;
    // If there was no match, that means that a quantifier
    // matched zero-length.  E.g., x (a)* y matched "xy".
    if (matchStart >= 0) {
        if (matchStart != matchLimit) {
            text.copy(matchStart, matchLimit, dest);
            outLen = matchLimit - matchStart;
        }
    }
    
    text.handleReplaceBetween(start, limit, EMPTY); // delete original text
    
    return outLen;
}

UnicodeString& StringMatcher::toReplacerPattern(UnicodeString& rule,
                                                UBool /*escapeUnprintable*/) const {
    // assert(segmentNumber > 0);
    rule.truncate(0);
    rule.append((UChar)0x0024 /*$*/);
    ICU_Utility::appendNumber(rule, segmentNumber, 10, 1);
    return rule;
}

 void StringMatcher::resetMatch() {
    matchStart = matchLimit = -1;
}

void StringMatcher::addReplacementSetTo(UnicodeSet& /*toUnionTo*/) const {
    // The output of this replacer varies; it is the source text between
    // matchStart and matchLimit.  Since this varies depending on the
    // input text, we can't compute it here.  We can either do nothing
    // or we can add ALL characters to the set.  It's probably more useful
    // to do nothing.
}

void StringMatcher::setData(const TransliterationRuleData* d) {
    data = d;
    int32_t i = 0;
    while (i<pattern.length()) {
        UChar32 c = pattern.char32At(i);
        UnicodeFunctor* f = data->lookup(c);
        if (f != NULL) {
            f->setData(data);
        }
        i += UTF_CHAR_LENGTH(c);
    }    
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

//eof
