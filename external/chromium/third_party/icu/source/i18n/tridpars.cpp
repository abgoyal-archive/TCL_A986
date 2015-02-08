
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "tridpars.h"
#include "hash.h"
#include "mutex.h"
#include "ucln_in.h"
#include "unicode/parsepos.h"
#include "unicode/translit.h"
#include "unicode/uchar.h"
#include "unicode/uniset.h"
#include "unicode/unistr.h"
#include "unicode/utrans.h"
#include "util.h"
#include "uvector.h"

U_NAMESPACE_BEGIN

static const UChar ID_DELIM    = 0x003B; // ;
static const UChar TARGET_SEP  = 0x002D; // -
static const UChar VARIANT_SEP = 0x002F; // /
static const UChar OPEN_REV    = 0x0028; // (
static const UChar CLOSE_REV   = 0x0029; // )

//static const UChar EMPTY[]     = {0}; // ""
static const UChar ANY[]       = {65,110,121,0}; // "Any"
static const UChar ANY_NULL[]  = {65,110,121,45,78,117,108,108,0}; // "Any-Null"

static const int32_t FORWARD = UTRANS_FORWARD;
static const int32_t REVERSE = UTRANS_REVERSE;

static Hashtable* SPECIAL_INVERSES = NULL;

static UMTX LOCK = 0;

TransliteratorIDParser::Specs::Specs(const UnicodeString& s, const UnicodeString& t,
                                     const UnicodeString& v, UBool sawS,
                                     const UnicodeString& f) {
    source = s;
    target = t;
    variant = v;
    sawSource = sawS;
    filter = f;
}

TransliteratorIDParser::SingleID::SingleID(const UnicodeString& c, const UnicodeString& b,
                                           const UnicodeString& f) {
    canonID = c;
    basicID = b;
    filter = f;
}

TransliteratorIDParser::SingleID::SingleID(const UnicodeString& c, const UnicodeString& b) {
    canonID = c;
    basicID = b;
}

Transliterator* TransliteratorIDParser::SingleID::createInstance() {
    Transliterator* t;
    if (basicID.length() == 0) {
        t = createBasicInstance(ANY_NULL, &canonID);
    } else {
        t = createBasicInstance(basicID, &canonID);
    }
    if (t != NULL) {
        if (filter.length() != 0) {
            UErrorCode ec = U_ZERO_ERROR;
            UnicodeSet *set = new UnicodeSet(filter, ec);
            if (U_FAILURE(ec)) {
                delete set;
            } else {
                t->adoptFilter(set);
            }
        }
    }
    return t;
}


TransliteratorIDParser::SingleID*
TransliteratorIDParser::parseSingleID(const UnicodeString& id, int32_t& pos,
                                      int32_t dir, UErrorCode& status) {

    int32_t start = pos;

    // The ID will be of the form A, A(), A(B), or (B), where
    // A and B are filter IDs.
    Specs* specsA = NULL;
    Specs* specsB = NULL;
    UBool sawParen = FALSE;

    // On the first pass, look for (B) or ().  If this fails, then
    // on the second pass, look for A, A(B), or A().
    for (int32_t pass=1; pass<=2; ++pass) {
        if (pass == 2) {
            specsA = parseFilterID(id, pos, TRUE);
            if (specsA == NULL) {
                pos = start;
                return NULL;
            }
        }
        if (ICU_Utility::parseChar(id, pos, OPEN_REV)) {
            sawParen = TRUE;
            if (!ICU_Utility::parseChar(id, pos, CLOSE_REV)) {
                specsB = parseFilterID(id, pos, TRUE);
                // Must close with a ')'
                if (specsB == NULL || !ICU_Utility::parseChar(id, pos, CLOSE_REV)) {
                    delete specsA;
                    pos = start;
                    return NULL;
                }
            }
            break;
        }
    }

    // Assemble return results
    SingleID* single;
    if (sawParen) {
        if (dir == FORWARD) {
            SingleID* b = specsToID(specsB, FORWARD);
            single = specsToID(specsA, FORWARD);
            // Null pointers check
            if (b == NULL || single == NULL) {
            	delete b;
            	delete single;
            	status = U_MEMORY_ALLOCATION_ERROR;
            	return NULL;
            }
            single->canonID.append(OPEN_REV)
                .append(b->canonID).append(CLOSE_REV);
            if (specsA != NULL) {
                single->filter = specsA->filter;
            }
            delete b;
        } else {
            SingleID* a = specsToID(specsA, FORWARD);
            single = specsToID(specsB, FORWARD);
            // Check for null pointer.
            if (a == NULL || single == NULL) {
            	delete a;
            	delete single;
            	status = U_MEMORY_ALLOCATION_ERROR;
            	return NULL;
            }
            single->canonID.append(OPEN_REV)
                .append(a->canonID).append(CLOSE_REV);
            if (specsB != NULL) {
                single->filter = specsB->filter;
            }
            delete a;
        }
    } else {
        // assert(specsA != NULL);
        if (dir == FORWARD) {
            single = specsToID(specsA, FORWARD);
        } else {
            single = specsToSpecialInverse(*specsA, status);
            if (single == NULL) {
                single = specsToID(specsA, REVERSE);
            }
        }
        // Check for NULL pointer
        if (single == NULL) {
        	status = U_MEMORY_ALLOCATION_ERROR;
        	return NULL;
        }
        single->filter = specsA->filter;
    }

    delete specsA;
    delete specsB;

    return single;
}

TransliteratorIDParser::SingleID*
TransliteratorIDParser::parseFilterID(const UnicodeString& id, int32_t& pos) {

    int32_t start = pos;

    Specs* specs = parseFilterID(id, pos, TRUE);
    if (specs == NULL) {
        pos = start;
        return NULL;
    }

    // Assemble return results
    SingleID* single = specsToID(specs, FORWARD);
    if (single != NULL) {
        single->filter = specs->filter;
    }
    delete specs;
    return single;
}

UnicodeSet* TransliteratorIDParser::parseGlobalFilter(const UnicodeString& id, int32_t& pos,
                                                      int32_t dir,
                                                      int32_t& withParens,
                                                      UnicodeString* canonID) {
    UnicodeSet* filter = NULL;
    int32_t start = pos;

    if (withParens == -1) {
        withParens = ICU_Utility::parseChar(id, pos, OPEN_REV) ? 1 : 0;
    } else if (withParens == 1) {
        if (!ICU_Utility::parseChar(id, pos, OPEN_REV)) {
            pos = start;
            return NULL;
        }
    }

    ICU_Utility::skipWhitespace(id, pos, TRUE);

    if (UnicodeSet::resemblesPattern(id, pos)) {
        ParsePosition ppos(pos);
        UErrorCode ec = U_ZERO_ERROR;
        filter = new UnicodeSet(id, ppos, USET_IGNORE_SPACE, NULL, ec);
        /* test for NULL */
        if (filter == 0) {
            pos = start;
            return 0;
        }
        if (U_FAILURE(ec)) {
            delete filter;
            pos = start;
            return NULL;
        }

        UnicodeString pattern;
        id.extractBetween(pos, ppos.getIndex(), pattern);
        pos = ppos.getIndex();

        if (withParens == 1 && !ICU_Utility::parseChar(id, pos, CLOSE_REV)) {
            pos = start;
            return NULL;
        }

        // In the forward direction, append the pattern to the
        // canonID.  In the reverse, insert it at zero, and invert
        // the presence of parens ("A" <-> "(A)").
        if (canonID != NULL) {
            if (dir == FORWARD) {
                if (withParens == 1) {
                    pattern.insert(0, OPEN_REV);
                    pattern.append(CLOSE_REV);
                }
                canonID->append(pattern).append(ID_DELIM);
            } else {
                if (withParens == 0) {
                    pattern.insert(0, OPEN_REV);
                    pattern.append(CLOSE_REV);
                }
                canonID->insert(0, pattern);
                canonID->insert(pattern.length(), ID_DELIM);
            }
        }
    }

    return filter;
}

U_CDECL_BEGIN
static void U_CALLCONV _deleteSingleID(void* obj) {
    delete (TransliteratorIDParser::SingleID*) obj;
}

static void U_CALLCONV _deleteTransliteratorTrIDPars(void* obj) {
    delete (Transliterator*) obj;
}
U_CDECL_END

UBool TransliteratorIDParser::parseCompoundID(const UnicodeString& id, int32_t dir,
                                              UnicodeString& canonID,
                                              UVector& list,
                                              UnicodeSet*& globalFilter) {
    UErrorCode ec = U_ZERO_ERROR;
    int32_t i;
    int32_t pos = 0;
    int32_t withParens = 1;
    list.removeAllElements();
    UnicodeSet* filter;
    globalFilter = NULL;
    canonID.truncate(0);

    // Parse leading global filter, if any
    withParens = 0; // parens disallowed
    filter = parseGlobalFilter(id, pos, dir, withParens, &canonID);
    if (filter != NULL) {
        if (!ICU_Utility::parseChar(id, pos, ID_DELIM)) {
            // Not a global filter; backup and resume
            canonID.truncate(0);
            pos = 0;
        }
        if (dir == FORWARD) {
            globalFilter = filter;
        } else {
            delete filter;
        }
        filter = NULL;
    }

    UBool sawDelimiter = TRUE;
    for (;;) {
        SingleID* single = parseSingleID(id, pos, dir, ec);
        if (single == NULL) {
            break;
        }
        if (dir == FORWARD) {
            list.addElement(single, ec);
        } else {
            list.insertElementAt(single, 0, ec);
        }
        if (U_FAILURE(ec)) {
            goto FAIL;
        }
        if (!ICU_Utility::parseChar(id, pos, ID_DELIM)) {
            sawDelimiter = FALSE;
            break;
        }
    }

    if (list.size() == 0) {
        goto FAIL;
    }

    // Construct canonical ID
    for (i=0; i<list.size(); ++i) {
        SingleID* single = (SingleID*) list.elementAt(i);
        canonID.append(single->canonID);
        if (i != (list.size()-1)) {
            canonID.append(ID_DELIM);
        }
    }

    // Parse trailing global filter, if any, and only if we saw
    // a trailing delimiter after the IDs.
    if (sawDelimiter) {
        withParens = 1; // parens required
        filter = parseGlobalFilter(id, pos, dir, withParens, &canonID);
        if (filter != NULL) {
            // Don't require trailing ';', but parse it if present
            ICU_Utility::parseChar(id, pos, ID_DELIM);

            if (dir == REVERSE) {
                globalFilter = filter;
            } else {
                delete filter;
            }
            filter = NULL;
        }
    }

    // Trailing unparsed text is a syntax error
    ICU_Utility::skipWhitespace(id, pos, TRUE);
    if (pos != id.length()) {
        goto FAIL;
    }

    return TRUE;

 FAIL:
    UObjectDeleter *save = list.setDeleter(_deleteSingleID);
    list.removeAllElements();
    list.setDeleter(save);
    delete globalFilter;
    globalFilter = NULL;
    return FALSE;
}

void TransliteratorIDParser::instantiateList(UVector& list,
                                                UErrorCode& ec) {
    UVector tlist(ec);
    if (U_FAILURE(ec)) {
        goto RETURN;
    }
    tlist.setDeleter(_deleteTransliteratorTrIDPars);

    Transliterator* t;
    int32_t i;
    for (i=0; i<=list.size(); ++i) { // [sic]: i<=list.size()
        // We run the loop too long by one, so we can
        // do an insert after the last element
        if (i==list.size()) {
            break;
        }

        SingleID* single = (SingleID*) list.elementAt(i);
        if (single->basicID.length() != 0) {
            t = single->createInstance();
            if (t == NULL) {
                ec = U_INVALID_ID;
                goto RETURN;
            }
            tlist.addElement(t, ec);
            if (U_FAILURE(ec)) {
                delete t;
                goto RETURN;
            }
        }
    }

    // An empty list is equivalent to a NULL transliterator.
    if (tlist.size() == 0) {
        t = createBasicInstance(ANY_NULL, NULL);
        if (t == NULL) {
            // Should never happen
            ec = U_INTERNAL_TRANSLITERATOR_ERROR;
        }
        tlist.addElement(t, ec);
        if (U_FAILURE(ec)) {
            delete t;
        }
    }

 RETURN:

    UObjectDeleter *save = list.setDeleter(_deleteSingleID);
    list.removeAllElements();

    if (U_SUCCESS(ec)) {
        list.setDeleter(_deleteTransliteratorTrIDPars);

        while (tlist.size() > 0) {
            t = (Transliterator*) tlist.orphanElementAt(0);
            list.addElement(t, ec);
            if (U_FAILURE(ec)) {
                delete t;
                list.removeAllElements();
                break;
            }
        }
    }

    list.setDeleter(save);
}

void TransliteratorIDParser::IDtoSTV(const UnicodeString& id,
                                     UnicodeString& source,
                                     UnicodeString& target,
                                     UnicodeString& variant,
                                     UBool& isSourcePresent) {
    source = ANY;
    target.truncate(0);
    variant.truncate(0);

    int32_t sep = id.indexOf(TARGET_SEP);
    int32_t var = id.indexOf(VARIANT_SEP);
    if (var < 0) {
        var = id.length();
    }
    isSourcePresent = FALSE;

    if (sep < 0) {
        // Form: T/V or T (or /V)
        id.extractBetween(0, var, target);
        id.extractBetween(var, id.length(), variant);
    } else if (sep < var) {
        // Form: S-T/V or S-T (or -T/V or -T)
        if (sep > 0) {
            id.extractBetween(0, sep, source);
            isSourcePresent = TRUE;
        }
        id.extractBetween(++sep, var, target);
        id.extractBetween(var, id.length(), variant);
    } else {
        // Form: (S/V-T or /V-T)
        if (var > 0) {
            id.extractBetween(0, var, source);
            isSourcePresent = TRUE;
        }
        id.extractBetween(var, sep++, variant);
        id.extractBetween(sep, id.length(), target);
    }

    if (variant.length() > 0) {
        variant.remove(0, 1);
    }
}

void TransliteratorIDParser::STVtoID(const UnicodeString& source,
                                     const UnicodeString& target,
                                     const UnicodeString& variant,
                                     UnicodeString& id) {
    id = source;
    if (id.length() == 0) {
        id = ANY;
    }
    id.append(TARGET_SEP).append(target);
    if (variant.length() != 0) {
        id.append(VARIANT_SEP).append(variant);
    }
    // NUL-terminate the ID string for getTerminatedBuffer.
    // This prevents valgrind and Purify warnings.
    id.append((UChar)0);
    id.truncate(id.length()-1);
}

void TransliteratorIDParser::registerSpecialInverse(const UnicodeString& target,
                                                    const UnicodeString& inverseTarget,
                                                    UBool bidirectional,
                                                    UErrorCode &status) {
    init(status);
    if (U_FAILURE(status)) {
        return;
    }

    // If target == inverseTarget then force bidirectional => FALSE
    if (bidirectional && 0==target.caseCompare(inverseTarget, U_FOLD_CASE_DEFAULT)) {
        bidirectional = FALSE;
    }

    umtx_init(&LOCK);
    Mutex lock(&LOCK);

    UnicodeString *tempus = new UnicodeString(inverseTarget);  // Used for null pointer check before usage.
    if (tempus == NULL) {
    	status = U_MEMORY_ALLOCATION_ERROR;
    	return;
    }
    SPECIAL_INVERSES->put(target, tempus, status);
    if (bidirectional) {
    	tempus = new UnicodeString(target);
    	if (tempus == NULL) {
    		status = U_MEMORY_ALLOCATION_ERROR;
    		return;
    	}
        SPECIAL_INVERSES->put(inverseTarget, tempus, status);
    }
}

//----------------------------------------------------------------
// Private implementation
//----------------------------------------------------------------

TransliteratorIDParser::Specs*
TransliteratorIDParser::parseFilterID(const UnicodeString& id, int32_t& pos,
                                      UBool allowFilter) {
    UnicodeString first;
    UnicodeString source;
    UnicodeString target;
    UnicodeString variant;
    UnicodeString filter;
    UChar delimiter = 0;
    int32_t specCount = 0;
    int32_t start = pos;

    // This loop parses one of the following things with each
    // pass: a filter, a delimiter character (either '-' or '/'),
    // or a spec (source, target, or variant).
    for (;;) {
        ICU_Utility::skipWhitespace(id, pos, TRUE);
        if (pos == id.length()) {
            break;
        }

        // Parse filters
        if (allowFilter && filter.length() == 0 &&
            UnicodeSet::resemblesPattern(id, pos)) {

            ParsePosition ppos(pos);
            UErrorCode ec = U_ZERO_ERROR;
            UnicodeSet set(id, ppos, USET_IGNORE_SPACE, NULL, ec);
            if (U_FAILURE(ec)) {
                pos = start;
                return NULL;
            }
            id.extractBetween(pos, ppos.getIndex(), filter);
            pos = ppos.getIndex();
            continue;
        }

        if (delimiter == 0) {
            UChar c = id.charAt(pos);
            if ((c == TARGET_SEP && target.length() == 0) ||
                (c == VARIANT_SEP && variant.length() == 0)) {
                delimiter = c;
                ++pos;
                continue;
            }
        }

        // We are about to try to parse a spec with no delimiter
        // when we can no longer do so (we can only do so at the
        // start); break.
        if (delimiter == 0 && specCount > 0) {
            break;
        }

        UnicodeString spec = ICU_Utility::parseUnicodeIdentifier(id, pos);
        if (spec.length() == 0) {
            // Note that if there was a trailing delimiter, we
            // consume it.  So Foo-, Foo/, Foo-Bar/, and Foo/Bar-
            // are legal.
            break;
        }

        switch (delimiter) {
        case 0:
            first = spec;
            break;
        case TARGET_SEP:
            target = spec;
            break;
        case VARIANT_SEP:
            variant = spec;
            break;
        }
        ++specCount;
        delimiter = 0;
    }

    // A spec with no prior character is either source or target,
    // depending on whether an explicit "-target" was seen.
    if (first.length() != 0) {
        if (target.length() == 0) {
            target = first;
        } else {
            source = first;
        }
    }

    // Must have either source or target
    if (source.length() == 0 && target.length() == 0) {
        pos = start;
        return NULL;
    }

    // Empty source or target defaults to ANY
    UBool sawSource = TRUE;
    if (source.length() == 0) {
        source = ANY;
        sawSource = FALSE;
    }
    if (target.length() == 0) {
        target = ANY;
    }

    return new Specs(source, target, variant, sawSource, filter);
}

TransliteratorIDParser::SingleID*
TransliteratorIDParser::specsToID(const Specs* specs, int32_t dir) {
    UnicodeString canonID;
    UnicodeString basicID;
    UnicodeString basicPrefix;
    if (specs != NULL) {
        UnicodeString buf;
        if (dir == FORWARD) {
            if (specs->sawSource) {
                buf.append(specs->source).append(TARGET_SEP);
            } else {
                basicPrefix = specs->source;
                basicPrefix.append(TARGET_SEP);
            }
            buf.append(specs->target);
        } else {
            buf.append(specs->target).append(TARGET_SEP).append(specs->source);
        }
        if (specs->variant.length() != 0) {
            buf.append(VARIANT_SEP).append(specs->variant);
        }
        basicID = basicPrefix;
        basicID.append(buf);
        if (specs->filter.length() != 0) {
            buf.insert(0, specs->filter);
        }
        canonID = buf;
    }
    return new SingleID(canonID, basicID);
}

TransliteratorIDParser::SingleID*
TransliteratorIDParser::specsToSpecialInverse(const Specs& specs, UErrorCode &status) {
    if (0!=specs.source.caseCompare(ANY, U_FOLD_CASE_DEFAULT)) {
        return NULL;
    }
    init(status);

    UnicodeString* inverseTarget;

    umtx_init(&LOCK);
    umtx_lock(&LOCK);
    inverseTarget = (UnicodeString*) SPECIAL_INVERSES->get(specs.target);
    umtx_unlock(&LOCK);

    if (inverseTarget != NULL) {
        // If the original ID contained "Any-" then make the
        // special inverse "Any-Foo"; otherwise make it "Foo".
        // So "Any-NFC" => "Any-NFD" but "NFC" => "NFD".
        UnicodeString buf;
        if (specs.filter.length() != 0) {
            buf.append(specs.filter);
        }
        if (specs.sawSource) {
            buf.append(ANY).append(TARGET_SEP);
        }
        buf.append(*inverseTarget);

        UnicodeString basicID(ANY);
        basicID.append(TARGET_SEP).append(*inverseTarget);

        if (specs.variant.length() != 0) {
            buf.append(VARIANT_SEP).append(specs.variant);
            basicID.append(VARIANT_SEP).append(specs.variant);
        }
        return new SingleID(buf, basicID);
    }
    return NULL;
}

Transliterator* TransliteratorIDParser::createBasicInstance(const UnicodeString& id, const UnicodeString* canonID) {
    return Transliterator::createBasicInstance(id, canonID);
}

void TransliteratorIDParser::init(UErrorCode &status) {
    if (SPECIAL_INVERSES != NULL) {
        return;
    }

    Hashtable* special_inverses = new Hashtable(TRUE, status);
    // Null pointer check
    if (special_inverses == NULL) {
    	status = U_MEMORY_ALLOCATION_ERROR;
    	return;
    }
    special_inverses->setValueDeleter(uhash_deleteUnicodeString);

    umtx_init(&LOCK);
    umtx_lock(&LOCK);
    if (SPECIAL_INVERSES == NULL) {
        SPECIAL_INVERSES = special_inverses;
        special_inverses = NULL;
    }
    umtx_unlock(&LOCK);
    delete special_inverses; /*null instance*/

    ucln_i18n_registerCleanup(UCLN_I18N_TRANSLITERATOR, utrans_transliterator_cleanup);
}

void TransliteratorIDParser::cleanup() {
    if (SPECIAL_INVERSES) {
        delete SPECIAL_INVERSES;
        SPECIAL_INVERSES = NULL;
    }
    umtx_destroy(&LOCK);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

//eof
