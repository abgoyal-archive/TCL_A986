
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/rep.h"
#include "unicode/unifilt.h"
#include "unicode/uniset.h"
#include "rbt_rule.h"
#include "rbt_data.h"
#include "cmemory.h"
#include "strmatch.h"
#include "strrepl.h"
#include "util.h"
#include "putilimp.h"

static const UChar FORWARD_OP[] = {32,62,32,0}; // " > "

U_NAMESPACE_BEGIN

TransliterationRule::TransliterationRule(const UnicodeString& input,
                                         int32_t anteContextPos, int32_t postContextPos,
                                         const UnicodeString& outputStr,
                                         int32_t cursorPosition, int32_t cursorOffset,
                                         UnicodeFunctor** segs,
                                         int32_t segsCount,
                                         UBool anchorStart, UBool anchorEnd,
                                         const TransliterationRuleData* theData,
                                         UErrorCode& status) :
    UMemory(),
    segments(0),
    data(theData) {

    if (U_FAILURE(status)) {
        return;
    }
    // Do range checks only when warranted to save time
    if (anteContextPos < 0) {
        anteContextLength = 0;
    } else {
        if (anteContextPos > input.length()) {
            // throw new IllegalArgumentException("Invalid ante context");
            status = U_ILLEGAL_ARGUMENT_ERROR;
            return;
        }
        anteContextLength = anteContextPos;
    }
    if (postContextPos < 0) {
        keyLength = input.length() - anteContextLength;
    } else {
        if (postContextPos < anteContextLength ||
            postContextPos > input.length()) {
            // throw new IllegalArgumentException("Invalid post context");
            status = U_ILLEGAL_ARGUMENT_ERROR;
            return;
        }
        keyLength = postContextPos - anteContextLength;
    }
    if (cursorPosition < 0) {
        cursorPosition = outputStr.length();
    } else if (cursorPosition > outputStr.length()) {
        // throw new IllegalArgumentException("Invalid cursor position");
        status = U_ILLEGAL_ARGUMENT_ERROR;
        return;
    }
    // We don't validate the segments array.  The caller must
    // guarantee that the segments are well-formed (that is, that
    // all $n references in the output refer to indices of this
    // array, and that no array elements are null).
    this->segments = segs;
    this->segmentsCount = segsCount;

    pattern = input;
    flags = 0;
    if (anchorStart) {
        flags |= ANCHOR_START;
    }
    if (anchorEnd) {
        flags |= ANCHOR_END;
    }

    anteContext = NULL;
    if (anteContextLength > 0) {
        anteContext = new StringMatcher(pattern, 0, anteContextLength,
                                        FALSE, *data);
        /* test for NULL */
        if (anteContext == 0) {
            status = U_MEMORY_ALLOCATION_ERROR;
            return;
        }
    }
    
    key = NULL;
    if (keyLength > 0) {
        key = new StringMatcher(pattern, anteContextLength, anteContextLength + keyLength,
                                FALSE, *data);
        /* test for NULL */
        if (key == 0) {
            status = U_MEMORY_ALLOCATION_ERROR;
            return;
        }
    }
    
    int32_t postContextLength = pattern.length() - keyLength - anteContextLength;
    postContext = NULL;
    if (postContextLength > 0) {
        postContext = new StringMatcher(pattern, anteContextLength + keyLength, pattern.length(),
                                        FALSE, *data);
        /* test for NULL */
        if (postContext == 0) {
            status = U_MEMORY_ALLOCATION_ERROR;
            return;
        }
    }

    this->output = new StringReplacer(outputStr, cursorPosition + cursorOffset, data);
    /* test for NULL */
    if (this->output == 0) {
        status = U_MEMORY_ALLOCATION_ERROR;
        return;
    }
}

TransliterationRule::TransliterationRule(TransliterationRule& other) :
    UMemory(other),
    anteContext(NULL),
    key(NULL),
    postContext(NULL),
    pattern(other.pattern),
    anteContextLength(other.anteContextLength),
    keyLength(other.keyLength),
    flags(other.flags),
    data(other.data) {

    segments = NULL;
    segmentsCount = 0;
    if (other.segmentsCount > 0) {
        segments = (UnicodeFunctor **)uprv_malloc(other.segmentsCount * sizeof(UnicodeFunctor *));
        uprv_memcpy(segments, other.segments, other.segmentsCount*sizeof(segments[0]));
    }

    if (other.anteContext != NULL) {
        anteContext = (StringMatcher*) other.anteContext->clone();
    }
    if (other.key != NULL) {
        key = (StringMatcher*) other.key->clone();
    }
    if (other.postContext != NULL) {
        postContext = (StringMatcher*) other.postContext->clone();
    }
    output = other.output->clone();
}

TransliterationRule::~TransliterationRule() {
    uprv_free(segments);
    delete anteContext;
    delete key;
    delete postContext;
    delete output;
}

int32_t TransliterationRule::getContextLength(void) const {
    return anteContextLength + ((flags & ANCHOR_START) ? 1 : 0);
}

int16_t TransliterationRule::getIndexValue() const {
    if (anteContextLength == pattern.length()) {
        // A pattern with just ante context {such as foo)>bar} can
        // match any key.
        return -1;
    }
    UChar32 c = pattern.char32At(anteContextLength);
    return (int16_t)(data->lookupMatcher(c) == NULL ? (c & 0xFF) : -1);
}

UBool TransliterationRule::matchesIndexValue(uint8_t v) const {
    // Delegate to the key, or if there is none, to the postContext.
    // If there is neither then we match any key; return true.
    UnicodeMatcher *m = (key != NULL) ? key : postContext;
    return (m != NULL) ? m->matchesIndexValue(v) : TRUE;
}

UBool TransliterationRule::masks(const TransliterationRule& r2) const {
    /* Rule r1 masks rule r2 if the string formed of the
     * antecontext, key, and postcontext overlaps in the following
     * way:
     *
     * r1:      aakkkpppp
     * r2:     aaakkkkkpppp
     *            ^
     * 
     * The strings must be aligned at the first character of the
     * key.  The length of r1 to the left of the alignment point
     * must be <= the length of r2 to the left; ditto for the
     * right.  The characters of r1 must equal (or be a superset
     * of) the corresponding characters of r2.  The superset
     * operation should be performed to check for UnicodeSet
     * masking.
     *
     * Anchors:  Two patterns that differ only in anchors only
     * mask one another if they are exactly equal, and r2 has
     * all the anchors r1 has (optionally, plus some).  Here Y
     * means the row masks the column, N means it doesn't.
     *
     *         ab   ^ab    ab$  ^ab$
     *   ab    Y     Y     Y     Y
     *  ^ab    N     Y     N     Y
     *   ab$   N     N     Y     Y
     *  ^ab$   N     N     N     Y
     *
     * Post context: {a}b masks ab, but not vice versa, since {a}b
     * matches everything ab matches, and {a}b matches {|a|}b but ab
     * does not.  Pre context is different (a{b} does not align with
     * ab).
     */

    /* LIMITATION of the current mask algorithm: Some rule
     * maskings are currently not detected.  For example,
     * "{Lu}]a>x" masks "A]a>y".  This can be added later. TODO
     */

    int32_t len = pattern.length();
    int32_t left = anteContextLength;
    int32_t left2 = r2.anteContextLength;
    int32_t right = len - left;
    int32_t right2 = r2.pattern.length() - left2;
    int32_t cachedCompare = r2.pattern.compare(left2 - left, len, pattern);

    // TODO Clean this up -- some logic might be combinable with the
    // next statement.

    // Test for anchor masking
    if (left == left2 && right == right2 &&
        keyLength <= r2.keyLength &&
        0 == cachedCompare) {
        // The following boolean logic implements the table above
        return (flags == r2.flags) ||
            (!(flags & ANCHOR_START) && !(flags & ANCHOR_END)) ||
            ((r2.flags & ANCHOR_START) && (r2.flags & ANCHOR_END));
    }

    return left <= left2 &&
        (right < right2 ||
         (right == right2 && keyLength <= r2.keyLength)) &&
         (0 == cachedCompare);
}

static inline int32_t posBefore(const Replaceable& str, int32_t pos) {
    return (pos > 0) ?
        pos - UTF_CHAR_LENGTH(str.char32At(pos-1)) :
        pos - 1;
}

static inline int32_t posAfter(const Replaceable& str, int32_t pos) {
    return (pos >= 0 && pos < str.length()) ?
        pos + UTF_CHAR_LENGTH(str.char32At(pos)) :
        pos + 1;
}

UMatchDegree TransliterationRule::matchAndReplace(Replaceable& text,
                                                  UTransPosition& pos,
                                                  UBool incremental) const {
    // Matching and replacing are done in one method because the
    // replacement operation needs information obtained during the
    // match.  Another way to do this is to have the match method
    // create a match result struct with relevant offsets, and to pass
    // this into the replace method.

    // ============================ MATCH ===========================

    // Reset segment match data
    if (segments != NULL) {
        for (int32_t i=0; i<segmentsCount; ++i) {
            ((StringMatcher*) segments[i])->resetMatch();
        }
    }

//    int32_t lenDelta, keyLimit;
    int32_t keyLimit;

    // ------------------------ Ante Context ------------------------

    // A mismatch in the ante context, or with the start anchor,
    // is an outright U_MISMATCH regardless of whether we are
    // incremental or not.
    int32_t oText; // offset into 'text'
//    int32_t newStart = 0;
    int32_t minOText;

    // Note (1): We process text in 16-bit code units, rather than
    // 32-bit code points.  This works because stand-ins are
    // always in the BMP and because we are doing a literal match
    // operation, which can be done 16-bits at a time.
    
    int32_t anteLimit = posBefore(text, pos.contextStart);

    UMatchDegree match;

    // Start reverse match at char before pos.start
    oText = posBefore(text, pos.start);

    if (anteContext != NULL) {
        match = anteContext->matches(text, oText, anteLimit, FALSE);
        if (match != U_MATCH) {
            return U_MISMATCH;
        }
    }

    minOText = posAfter(text, oText);

    // ------------------------ Start Anchor ------------------------
    
    if (((flags & ANCHOR_START) != 0) && oText != anteLimit) {
        return U_MISMATCH;
    }

    // -------------------- Key and Post Context --------------------
    
    oText = pos.start;

    if (key != NULL) {
        match = key->matches(text, oText, pos.limit, incremental);
        if (match != U_MATCH) {
            return match;
        }
    }

    keyLimit = oText;

    if (postContext != NULL) {
        if (incremental && keyLimit == pos.limit) {
            // The key matches just before pos.limit, and there is
            // a postContext.  Since we are in incremental mode,
            // we must assume more characters may be inserted at
            // pos.limit -- this is a partial match.
            return U_PARTIAL_MATCH;
        }

        match = postContext->matches(text, oText, pos.contextLimit, incremental);
        if (match != U_MATCH) {
            return match;
        }
    }
    
    // ------------------------- Stop Anchor ------------------------
    
    if (((flags & ANCHOR_END)) != 0) {
        if (oText != pos.contextLimit) {
            return U_MISMATCH;
        }
        if (incremental) {
            return U_PARTIAL_MATCH;
        }
    }
    
    // =========================== REPLACE ==========================

    // We have a full match.  The key is between pos.start and
    // keyLimit.

    int32_t newStart;
    int32_t newLength = output->toReplacer()->replace(text, pos.start, keyLimit, newStart);
    int32_t lenDelta = newLength - (keyLimit - pos.start);

    oText += lenDelta;
    pos.limit += lenDelta;
    pos.contextLimit += lenDelta;
    // Restrict new value of start to [minOText, min(oText, pos.limit)].
    pos.start = uprv_max(minOText, uprv_min(uprv_min(oText, pos.limit), newStart));
    return U_MATCH;
}

UnicodeString& TransliterationRule::toRule(UnicodeString& rule,
                                           UBool escapeUnprintable) const {

    // Accumulate special characters (and non-specials following them)
    // into quoteBuf.  Append quoteBuf, within single quotes, when
    // a non-quoted element must be inserted.
    UnicodeString str, quoteBuf;

    // Do not emit the braces '{' '}' around the pattern if there
    // is neither anteContext nor postContext.
    UBool emitBraces =
        (anteContext != NULL) || (postContext != NULL);

    // Emit start anchor
    if ((flags & ANCHOR_START) != 0) {
        rule.append((UChar)94/*^*/);
    }

    // Emit the input pattern
    ICU_Utility::appendToRule(rule, anteContext, escapeUnprintable, quoteBuf);

    if (emitBraces) {
        ICU_Utility::appendToRule(rule, (UChar) 0x007B /*{*/, TRUE, escapeUnprintable, quoteBuf);
    }

    ICU_Utility::appendToRule(rule, key, escapeUnprintable, quoteBuf);

    if (emitBraces) {
        ICU_Utility::appendToRule(rule, (UChar) 0x007D /*}*/, TRUE, escapeUnprintable, quoteBuf);
    }

    ICU_Utility::appendToRule(rule, postContext, escapeUnprintable, quoteBuf);

    // Emit end anchor
    if ((flags & ANCHOR_END) != 0) {
        rule.append((UChar)36/*$*/);
    }

    ICU_Utility::appendToRule(rule, FORWARD_OP, TRUE, escapeUnprintable, quoteBuf);

    // Emit the output pattern

    ICU_Utility::appendToRule(rule, output->toReplacer()->toReplacerPattern(str, escapeUnprintable),
                              TRUE, escapeUnprintable, quoteBuf);

    ICU_Utility::appendToRule(rule, (UChar) 0x003B /*;*/, TRUE, escapeUnprintable, quoteBuf);

    return rule;
}

void TransliterationRule::setData(const TransliterationRuleData* d) {
    data = d;
    if (anteContext != NULL) anteContext->setData(d);
    if (postContext != NULL) postContext->setData(d);
    if (key != NULL) key->setData(d);
    // assert(output != NULL);
    output->setData(d);
    // Don't have to do segments since they are in the context or key
}

void TransliterationRule::addSourceSetTo(UnicodeSet& toUnionTo) const {
    int32_t limit = anteContextLength + keyLength;
    for (int32_t i=anteContextLength; i<limit; ) {
        UChar32 ch = pattern.char32At(i);
        i += UTF_CHAR_LENGTH(ch);
        const UnicodeMatcher* matcher = data->lookupMatcher(ch);
        if (matcher == NULL) {
            toUnionTo.add(ch);
        } else {
            matcher->addMatchSetTo(toUnionTo);
        }
    }
}

void TransliterationRule::addTargetSetTo(UnicodeSet& toUnionTo) const {
    output->toReplacer()->addReplacementSetTo(toUnionTo);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

//eof
