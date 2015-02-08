
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/putil.h"
#include "unicode/translit.h"
#include "unicode/locid.h"
#include "unicode/msgfmt.h"
#include "unicode/rep.h"
#include "unicode/resbund.h"
#include "unicode/unifilt.h"
#include "unicode/uniset.h"
#include "unicode/uscript.h"
#include "unicode/strenum.h"
#include "cpdtrans.h"
#include "nultrans.h"
#include "rbt_data.h"
#include "rbt_pars.h"
#include "rbt.h"
#include "transreg.h"
#include "name2uni.h"
#include "nortrans.h"
#include "remtrans.h"
#include "titletrn.h"
#include "tolowtrn.h"
#include "toupptrn.h"
#include "uni2name.h"
#include "brktrans.h"
#include "esctrn.h"
#include "unesctrn.h"
#include "tridpars.h"
#include "anytrans.h"
#include "util.h"
#include "hash.h"
#include "mutex.h"
#include "ucln_in.h"
#include "uassert.h"
#include "cmemory.h"
#include "cstring.h"
#include "uinvchar.h"

static const UChar TARGET_SEP  = 0x002D; /*-*/
static const UChar ID_DELIM    = 0x003B; /*;*/
static const UChar VARIANT_SEP = 0x002F; // '/'

static const char RB_DISPLAY_NAME_PREFIX[] = "%Translit%%";

static const char RB_SCRIPT_DISPLAY_NAME_PREFIX[] = "%Translit%";

static const char RB_DISPLAY_NAME_PATTERN[] = "TransliteratorNamePattern";

static const char RB_RULE_BASED_IDS[] = "RuleBasedTransliteratorIDs";

static UMTX registryMutex = 0;

static U_NAMESPACE_QUALIFIER TransliteratorRegistry* registry = 0;

// Macro to check/initialize the registry. ONLY USE WITHIN
// MUTEX. Avoids function call when registry is initialized.
#define HAVE_REGISTRY(status) (registry!=0 || initializeRegistry(status))

// Empty string
static const UChar EMPTY[] = {0}; //""

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_ABSTRACT_RTTI_IMPLEMENTATION(Transliterator)

static inline UBool positionIsValid(UTransPosition& index, int32_t len) {
    return !(index.contextStart < 0 ||
             index.start < index.contextStart ||
             index.limit < index.start ||
             index.contextLimit < index.limit ||
             len < index.contextLimit);
}

Transliterator::Transliterator(const UnicodeString& theID,
                               UnicodeFilter* adoptedFilter) :
    UObject(), ID(theID), filter(adoptedFilter),
    maximumContextLength(0)
{
    // NUL-terminate the ID string, which is a non-aliased copy.
    ID.append((UChar)0);
    ID.truncate(ID.length()-1);
}

Transliterator::~Transliterator() {
    if (filter) {
        delete filter;
    }
}

Transliterator::Transliterator(const Transliterator& other) :
    UObject(other), ID(other.ID), filter(0),
    maximumContextLength(other.maximumContextLength)
{
    // NUL-terminate the ID string, which is a non-aliased copy.
    ID.append((UChar)0);
    ID.truncate(ID.length()-1);

    if (other.filter != 0) {
        // We own the filter, so we must have our own copy
        filter = (UnicodeFilter*) other.filter->clone();
    }
}

Transliterator* Transliterator::clone() const {
    return NULL;
}

Transliterator& Transliterator::operator=(const Transliterator& other) {
    ID = other.ID;
    // NUL-terminate the ID string
    ID.getTerminatedBuffer();

    maximumContextLength = other.maximumContextLength;
    adoptFilter((other.filter == 0) ? 0 : (UnicodeFilter*) other.filter->clone());
    return *this;
}

int32_t Transliterator::transliterate(Replaceable& text,
                                      int32_t start, int32_t limit) const {
    if (start < 0 ||
        limit < start ||
        text.length() < limit) {
        return -1;
    }

    UTransPosition offsets;
    offsets.contextStart= start;
    offsets.contextLimit = limit;
    offsets.start = start;
    offsets.limit = limit;
    filteredTransliterate(text, offsets, FALSE, TRUE);
    return offsets.limit;
}

void Transliterator::transliterate(Replaceable& text) const {
    transliterate(text, 0, text.length());
}

void Transliterator::transliterate(Replaceable& text,
                                   UTransPosition& index,
                                   const UnicodeString& insertion,
                                   UErrorCode &status) const {
    _transliterate(text, index, &insertion, status);
}

void Transliterator::transliterate(Replaceable& text,
                                   UTransPosition& index,
                                   UChar32 insertion,
                                   UErrorCode& status) const {
    UnicodeString str(insertion);
    _transliterate(text, index, &str, status);
}

void Transliterator::transliterate(Replaceable& text,
                                   UTransPosition& index,
                                   UErrorCode& status) const {
    _transliterate(text, index, 0, status);
}

void Transliterator::finishTransliteration(Replaceable& text,
                                           UTransPosition& index) const {
    if (!positionIsValid(index, text.length())) {
        return;
    }

    filteredTransliterate(text, index, FALSE, TRUE);
}

void Transliterator::_transliterate(Replaceable& text,
                                    UTransPosition& index,
                                    const UnicodeString* insertion,
                                    UErrorCode &status) const {
    if (U_FAILURE(status)) {
        return;
    }

    if (!positionIsValid(index, text.length())) {
        status = U_ILLEGAL_ARGUMENT_ERROR;
        return;
    }

//    int32_t originalStart = index.contextStart;
    if (insertion != 0) {
        text.handleReplaceBetween(index.limit, index.limit, *insertion);
        index.limit += insertion->length();
        index.contextLimit += insertion->length();
    }

    if (index.limit > 0 &&
        UTF_IS_LEAD(text.charAt(index.limit - 1))) {
        // Oops, there is a dangling lead surrogate in the buffer.
        // This will break most transliterators, since they will
        // assume it is part of a pair.  Don't transliterate until
        // more text comes in.
        return;
    }

    filteredTransliterate(text, index, TRUE, TRUE);

#if 0
    // TODO
    // I CAN'T DO what I'm attempting below now that the Kleene star
    // operator is supported.  For example, in the rule

    //   ([:Lu:]+) { x } > $1;

    // what is the maximum context length?  getMaximumContextLength()
    // will return 1, but this is just the length of the ante context
    // part of the pattern string -- 1 character, which is a standin
    // for a Quantifier, which contains a StringMatcher, which
    // contains a UnicodeSet.

    // There is a complicated way to make this work again, and that's
    // to add a "maximum left context" protocol into the
    // UnicodeMatcher hierarchy.  At present I'm not convinced this is
    // worth it.

    // ---

    // The purpose of the code below is to keep the context small
    // while doing incremental transliteration.  When part of the left
    // context (between contextStart and start) is no longer needed,
    // we try to advance contextStart past that portion.  We use the
    // maximum context length to do so.
    int32_t newCS = index.start;
    int32_t n = getMaximumContextLength();
    while (newCS > originalStart && n-- > 0) {
        --newCS;
        newCS -= UTF_CHAR_LENGTH(text.char32At(newCS)) - 1;
    }
    index.contextStart = uprv_max(newCS, originalStart);
#endif
}

void Transliterator::filteredTransliterate(Replaceable& text,
                                           UTransPosition& index,
                                           UBool incremental,
                                           UBool rollback) const {
    // Short circuit path for transliterators with no filter in
    // non-incremental mode.
    if (filter == 0 && !rollback) {
        handleTransliterate(text, index, incremental);
        return;
    }

    //----------------------------------------------------------------------
    // This method processes text in two groupings:
    //
    // RUNS -- A run is a contiguous group of characters which are contained
    // in the filter for this transliterator (filter.contains(ch) == TRUE).
    // Text outside of runs may appear as context but it is not modified.
    // The start and limit Position values are narrowed to each run.
    //
    // PASSES (incremental only) -- To make incremental mode work correctly,
    // each run is broken up into n passes, where n is the length (in code
    // points) of the run.  Each pass contains the first n characters.  If a
    // pass is completely transliterated, it is committed, and further passes
    // include characters after the committed text.  If a pass is blocked,
    // and does not transliterate completely, then this method rolls back
    // the changes made during the pass, extends the pass by one code point,
    // and tries again.
    //----------------------------------------------------------------------
    
    // globalLimit is the limit value for the entire operation.  We
    // set index.limit to the end of each unfiltered run before
    // calling handleTransliterate(), so we need to maintain the real
    // value of index.limit here.  After each transliteration, we
    // update globalLimit for insertions or deletions that have
    // happened.
    int32_t globalLimit = index.limit;
    
    // If there is a non-null filter, then break the input text up.  Say the
    // input text has the form:
    //   xxxabcxxdefxx
    // where 'x' represents a filtered character (filter.contains('x') ==
    // false).  Then we break this up into:
    //   xxxabc xxdef xx
    // Each pass through the loop consumes a run of filtered
    // characters (which are ignored) and a subsequent run of
    // unfiltered characters (which are transliterated).
    
    for (;;) {

        if (filter != NULL) {
            // Narrow the range to be transliterated to the first segment
            // of unfiltered characters at or after index.start.

            // Advance past filtered chars
            UChar32 c;
            while (index.start < globalLimit &&
                   !filter->contains(c=text.char32At(index.start))) {
                index.start += UTF_CHAR_LENGTH(c);
            }

            // Find the end of this run of unfiltered chars
            index.limit = index.start;
            while (index.limit < globalLimit &&
                   filter->contains(c=text.char32At(index.limit))) {
                index.limit += UTF_CHAR_LENGTH(c);
            }
        }

        // Check to see if the unfiltered run is empty.  This only
        // happens at the end of the string when all the remaining
        // characters are filtered.
        if (index.limit == index.start) {
            // assert(index.start == globalLimit);
            break;
        }

        // Is this run incremental?  If there is additional
        // filtered text (if limit < globalLimit) then we pass in
        // an incremental value of FALSE to force the subclass to
        // complete the transliteration for this run.
        UBool isIncrementalRun =
            (index.limit < globalLimit ? FALSE : incremental);
        
        int32_t delta;

        // Implement rollback.  To understand the need for rollback,
        // consider the following transliterator:
        //
        //  "t" is "a > A;"
        //  "u" is "A > b;"
        //  "v" is a compound of "t; NFD; u" with a filter [:Ll:]
        //
        // Now apply "c" to the input text "a".  The result is "b".  But if
        // the transliteration is done incrementally, then the NFD holds
        // things up after "t" has already transformed "a" to "A".  When
        // finishTransliterate() is called, "A" is _not_ processed because
        // it gets excluded by the [:Ll:] filter, and the end result is "A"
        // -- incorrect.  The problem is that the filter is applied to a
        // partially-transliterated result, when we only want it to apply to
        // input text.  Although this example hinges on a compound
        // transliterator containing NFD and a specific filter, it can
        // actually happen with any transliterator which may do a partial
        // transformation in incremental mode into characters outside its
        // filter.
        //
        // To handle this, when in incremental mode we supply characters to
        // handleTransliterate() in several passes.  Each pass adds one more
        // input character to the input text.  That is, for input "ABCD", we
        // first try "A", then "AB", then "ABC", and finally "ABCD".  If at
        // any point we block (upon return, start < limit) then we roll
        // back.  If at any point we complete the run (upon return start ==
        // limit) then we commit that run.

        if (rollback && isIncrementalRun) {

            int32_t runStart = index.start;
            int32_t runLimit = index.limit;
            int32_t runLength =  runLimit - runStart;

            // Make a rollback copy at the end of the string
            int32_t rollbackOrigin = text.length();
            text.copy(runStart, runLimit, rollbackOrigin);

            // Variables reflecting the commitment of completely
            // transliterated text.  passStart is the runStart, advanced
            // past committed text.  rollbackStart is the rollbackOrigin,
            // advanced past rollback text that corresponds to committed
            // text.
            int32_t passStart = runStart;
            int32_t rollbackStart = rollbackOrigin;

            // The limit for each pass; we advance by one code point with
            // each iteration.
            int32_t passLimit = index.start;

            // Total length, in 16-bit code units, of uncommitted text.
            // This is the length to be rolled back.
            int32_t uncommittedLength = 0;

            // Total delta (change in length) for all passes
            int32_t totalDelta = 0;

            // PASS MAIN LOOP -- Start with a single character, and extend
            // the text by one character at a time.  Roll back partial
            // transliterations and commit complete transliterations.
            for (;;) {
                // Length of additional code point, either one or two
                int32_t charLength =
                    UTF_CHAR_LENGTH(text.char32At(passLimit));
                passLimit += charLength;
                if (passLimit > runLimit) {
                    break;
                }
                uncommittedLength += charLength;

                index.limit = passLimit;

                // Delegate to subclass for actual transliteration.  Upon
                // return, start will be updated to point after the
                // transliterated text, and limit and contextLimit will be
                // adjusted for length changes.
                handleTransliterate(text, index, TRUE);

                delta = index.limit - passLimit; // change in length

                // We failed to completely transliterate this pass.
                // Roll back the text.  Indices remain unchanged; reset
                // them where necessary.
                if (index.start != index.limit) {
                    // Find the rollbackStart, adjusted for length changes
                    // and the deletion of partially transliterated text.
                    int32_t rs = rollbackStart + delta - (index.limit - passStart);

                    // Delete the partially transliterated text
                    text.handleReplaceBetween(passStart, index.limit, EMPTY);

                    // Copy the rollback text back
                    text.copy(rs, rs + uncommittedLength, passStart);

                    // Restore indices to their original values
                    index.start = passStart;
                    index.limit = passLimit;
                    index.contextLimit -= delta;
                }

                // We did completely transliterate this pass.  Update the
                // commit indices to record how far we got.  Adjust indices
                // for length change.
                else {
                    // Move the pass indices past the committed text.
                    passStart = passLimit = index.start;

                    // Adjust the rollbackStart for length changes and move
                    // it past the committed text.  All characters we've
                    // processed to this point are committed now, so zero
                    // out the uncommittedLength.
                    rollbackStart += delta + uncommittedLength;
                    uncommittedLength = 0;

                    // Adjust indices for length changes.
                    runLimit += delta;
                    totalDelta += delta;
                }
            }

            // Adjust overall limit and rollbackOrigin for insertions and
            // deletions.  Don't need to worry about contextLimit because
            // handleTransliterate() maintains that.
            rollbackOrigin += totalDelta;
            globalLimit += totalDelta;

            // Delete the rollback copy
            text.handleReplaceBetween(rollbackOrigin, rollbackOrigin + runLength, EMPTY);

            // Move start past committed text
            index.start = passStart;
        }

        else {
            // Delegate to subclass for actual transliteration.
            int32_t limit = index.limit;
            handleTransliterate(text, index, isIncrementalRun);
            delta = index.limit - limit; // change in length

            // In a properly written transliterator, start == limit after
            // handleTransliterate() returns when incremental is false.
            // Catch cases where the subclass doesn't do this, and throw
            // an exception.  (Just pinning start to limit is a bad idea,
            // because what's probably happening is that the subclass
            // isn't transliterating all the way to the end, and it should
            // in non-incremental mode.)
            if (!incremental && index.start != index.limit) {
                // We can't throw an exception, so just fudge things
                index.start = index.limit;
            }

            // Adjust overall limit for insertions/deletions.  Don't need
            // to worry about contextLimit because handleTransliterate()
            // maintains that.
            globalLimit += delta;
        }

        if (filter == NULL || isIncrementalRun) {
            break;
        }

        // If we did completely transliterate this
        // run, then repeat with the next unfiltered run.
    }

    // Start is valid where it is.  Limit needs to be put back where
    // it was, modulo adjustments for deletions/insertions.
    index.limit = globalLimit;
}

void Transliterator::filteredTransliterate(Replaceable& text,
                                           UTransPosition& index,
                                           UBool incremental) const {
    filteredTransliterate(text, index, incremental, FALSE);
}

void Transliterator::setMaximumContextLength(int32_t maxContextLength) {
    maximumContextLength = maxContextLength;
}

const UnicodeString& Transliterator::getID(void) const {
    return ID;
}

UnicodeString& U_EXPORT2 Transliterator::getDisplayName(const UnicodeString& ID,
                                              UnicodeString& result) {
    return getDisplayName(ID, Locale::getDefault(), result);
}

UnicodeString& U_EXPORT2 Transliterator::getDisplayName(const UnicodeString& id,
                                              const Locale& inLocale,
                                              UnicodeString& result) {
    UErrorCode status = U_ZERO_ERROR;

    ResourceBundle bundle(U_ICUDATA_TRANSLIT, inLocale, status);

    // Suspend checking status until later...

    result.truncate(0);

    // Normalize the ID
    UnicodeString source, target, variant;
    UBool sawSource;
    TransliteratorIDParser::IDtoSTV(id, source, target, variant, sawSource);
    if (target.length() < 1) {
        // No target; malformed id
        return result;
    }
    if (variant.length() > 0) { // Change "Foo" to "/Foo"
        variant.insert(0, VARIANT_SEP);
    }
    UnicodeString ID(source);
    ID.append(TARGET_SEP).append(target).append(variant);

    // build the char* key
    if (uprv_isInvariantUString(ID.getBuffer(), ID.length())) {
        char key[200];
        uprv_strcpy(key, RB_DISPLAY_NAME_PREFIX);
        int32_t length=(int32_t)uprv_strlen(RB_DISPLAY_NAME_PREFIX);
        ID.extract(0, (int32_t)(sizeof(key)-length), key+length, (int32_t)(sizeof(key)-length), US_INV);

        // Try to retrieve a UnicodeString from the bundle.
        UnicodeString resString = bundle.getStringEx(key, status);

        if (U_SUCCESS(status) && resString.length() != 0) {
            return result = resString; // [sic] assign & return
        }

#if !UCONFIG_NO_FORMATTING
        // We have failed to get a name from the locale data.  This is
        // typical, since most transliterators will not have localized
        // name data.  The next step is to retrieve the MessageFormat
        // pattern from the locale data and to use it to synthesize the
        // name from the ID.

        status = U_ZERO_ERROR;
        resString = bundle.getStringEx(RB_DISPLAY_NAME_PATTERN, status);

        if (U_SUCCESS(status) && resString.length() != 0) {
            MessageFormat msg(resString, inLocale, status);
            // Suspend checking status until later...

            // We pass either 2 or 3 Formattable objects to msg.
            Formattable args[3];
            int32_t nargs;
            args[0].setLong(2); // # of args to follow
            args[1].setString(source);
            args[2].setString(target);
            nargs = 3;

            // Use display names for the scripts, if they exist
            UnicodeString s;
            length=(int32_t)uprv_strlen(RB_SCRIPT_DISPLAY_NAME_PREFIX);
            for (int j=1; j<=2; ++j) {
                status = U_ZERO_ERROR;
                uprv_strcpy(key, RB_SCRIPT_DISPLAY_NAME_PREFIX);
                args[j].getString(s);
                if (uprv_isInvariantUString(s.getBuffer(), s.length())) {
                    s.extract(0, sizeof(key)-length-1, key+length, (int32_t)sizeof(key)-length-1, US_INV);

                    resString = bundle.getStringEx(key, status);

                    if (U_SUCCESS(status)) {
                        args[j] = resString;
                    }
                }
            }

            status = U_ZERO_ERROR;
            FieldPosition pos; // ignored by msg
            msg.format(args, nargs, result, pos, status);
            if (U_SUCCESS(status)) {
                result.append(variant);
                return result;
            }
        }
#endif
    }

    // We should not reach this point unless there is something
    // wrong with the build or the RB_DISPLAY_NAME_PATTERN has
    // been deleted from the root RB_LOCALE_ELEMENTS resource.
    result = ID;
    return result;
}

const UnicodeFilter* Transliterator::getFilter(void) const {
    return filter;
}

UnicodeFilter* Transliterator::orphanFilter(void) {
    UnicodeFilter *result = filter;
    filter = NULL;
    return result;
}

void Transliterator::adoptFilter(UnicodeFilter* filterToAdopt) {
    delete filter;
    filter = filterToAdopt;
}

Transliterator* Transliterator::createInverse(UErrorCode& status) const {
    UParseError parseError;
    return Transliterator::createInstance(ID, UTRANS_REVERSE,parseError,status);
}

Transliterator* U_EXPORT2
Transliterator::createInstance(const UnicodeString& ID,
                                UTransDirection dir,
                                UErrorCode& status)
{
    UParseError parseError;
    return createInstance(ID, dir, parseError, status);
}

Transliterator* U_EXPORT2
Transliterator::createInstance(const UnicodeString& ID,
                                UTransDirection dir,
                                UParseError& parseError,
                                UErrorCode& status)
{
    if (U_FAILURE(status)) {
        return 0;
    }

    UnicodeString canonID;
    UVector list(status);
    if (U_FAILURE(status)) {
        return NULL;
    }

    UnicodeSet* globalFilter;
    // TODO add code for parseError...currently unused, but
    // later may be used by parsing code...
    if (!TransliteratorIDParser::parseCompoundID(ID, dir, canonID, list, globalFilter)) {
        status = U_INVALID_ID;
        return NULL;
    }
    
    TransliteratorIDParser::instantiateList(list, status);
    if (U_FAILURE(status)) {
        return NULL;
    }
    
    U_ASSERT(list.size() > 0);
    Transliterator* t = NULL;
    
    if (list.size() > 1 || canonID.indexOf(ID_DELIM) >= 0) {
        // [NOTE: If it's a compoundID, we instantiate a CompoundTransliterator even if it only
        // has one child transliterator.  This is so that toRules() will return the right thing
        // (without any inactive ID), but our main ID still comes out correct.  That is, if we
        // instantiate "(Lower);Latin-Greek;", we want the rules to come out as "::Latin-Greek;"
        // even though the ID is "(Lower);Latin-Greek;".
        t = new CompoundTransliterator(list, parseError, status);
    }
    else {
        t = (Transliterator*)list.elementAt(0);
    }
    // Check null pointer
    if (t != NULL) {
        t->setID(canonID);
        if (globalFilter != NULL) {
            t->adoptFilter(globalFilter);
        }
    }
    else if (U_SUCCESS(status)) {
        status = U_MEMORY_ALLOCATION_ERROR;
    }
    return t;
}

Transliterator* Transliterator::createBasicInstance(const UnicodeString& id,
                                                    const UnicodeString* canon) {
    UParseError pe;
    UErrorCode ec = U_ZERO_ERROR;
    TransliteratorAlias* alias = 0;
    Transliterator* t = 0;

    umtx_lock(&registryMutex);
    if (HAVE_REGISTRY(ec)) {
        t = registry->get(id, alias, ec);
    }
    umtx_unlock(&registryMutex);

    if (U_FAILURE(ec)) {
        delete t;
        delete alias;
        return 0;
    }

    // We may have not gotten a transliterator:  Because we can't
    // instantiate a transliterator from inside TransliteratorRegistry::
    // get() (that would deadlock), we sometimes pass back an alias.  This
    // contains the data we need to finish the instantiation outside the
    // registry mutex.  The alias may, in turn, generate another alias, so
    // we handle aliases in a loop.  The max times through the loop is two.
    // [alan]
    while (alias != 0) {
        U_ASSERT(t==0);
        // Rule-based aliases are handled with TransliteratorAlias::
        // parse(), followed by TransliteratorRegistry::reget().
        // Other aliases are handled with TransliteratorAlias::create().
        if (alias->isRuleBased()) {
            // Step 1. parse
            TransliteratorParser parser(ec);
            alias->parse(parser, pe, ec);
            delete alias;
            alias = 0;

            // Step 2. reget
            umtx_lock(&registryMutex);
            if (HAVE_REGISTRY(ec)) {
                t = registry->reget(id, parser, alias, ec);
            }
            umtx_unlock(&registryMutex);

            // Step 3. Loop back around!
        } else {
            t = alias->create(pe, ec);
            delete alias;
            alias = 0;
            break;
        }
        if (U_FAILURE(ec)) {
            delete t;
            delete alias;
            t = NULL;
            break;
        }
    }

    if (t != NULL && canon != NULL) {
        t->setID(*canon);
    }

    return t;
}

Transliterator* U_EXPORT2
Transliterator::createFromRules(const UnicodeString& ID,
                                const UnicodeString& rules,
                                UTransDirection dir,
                                UParseError& parseError,
                                UErrorCode& status)
{
    Transliterator* t = NULL;

    TransliteratorParser parser(status);
    parser.parse(rules, dir, parseError, status);

    if (U_FAILURE(status)) {
        return 0;
    }

    // NOTE: The logic here matches that in TransliteratorRegistry.
    if (parser.idBlockVector.size() == 0 && parser.dataVector.size() == 0) {
        t = new NullTransliterator();
    }
    else if (parser.idBlockVector.size() == 0 && parser.dataVector.size() == 1) {
        t = new RuleBasedTransliterator(ID, (TransliterationRuleData*)parser.dataVector.orphanElementAt(0), TRUE);
    }
    else if (parser.idBlockVector.size() == 1 && parser.dataVector.size() == 0) {
        // idBlock, no data -- this is an alias.  The ID has
        // been munged from reverse into forward mode, if
        // necessary, so instantiate the ID in the forward
        // direction.
        if (parser.compoundFilter != NULL) {
            UnicodeString filterPattern;
            parser.compoundFilter->toPattern(filterPattern, FALSE);
            t = createInstance(filterPattern + UnicodeString(ID_DELIM)
                    + *((UnicodeString*)parser.idBlockVector.elementAt(0)), UTRANS_FORWARD, parseError, status);
        }
        else
            t = createInstance(*((UnicodeString*)parser.idBlockVector.elementAt(0)), UTRANS_FORWARD, parseError, status);


        if (t != NULL) {
            t->setID(ID);
        }
    }
    else {
        UVector transliterators(status);
        int32_t passNumber = 1;

        int32_t limit = parser.idBlockVector.size();
        if (parser.dataVector.size() > limit)
            limit = parser.dataVector.size();

        for (int32_t i = 0; i < limit; i++) {
            if (i < parser.idBlockVector.size()) {
                UnicodeString* idBlock = (UnicodeString*)parser.idBlockVector.elementAt(i);
                if (!idBlock->isEmpty()) {
                    Transliterator* temp = createInstance(*idBlock, UTRANS_FORWARD, parseError, status);
                    if (temp != NULL && temp->getDynamicClassID() != NullTransliterator::getStaticClassID())
                        transliterators.addElement(temp, status);
                    else
                        delete temp;
                }
            }
            if (!parser.dataVector.isEmpty()) {
                TransliterationRuleData* data = (TransliterationRuleData*)parser.dataVector.orphanElementAt(0);
                RuleBasedTransliterator* temprbt = new RuleBasedTransliterator(UnicodeString(CompoundTransliterator::PASS_STRING) + (passNumber++),
                        data, TRUE);
                // Check if NULL before adding it to transliterators to avoid future usage of NULL pointer.
                if (temprbt == NULL) {
                	status = U_MEMORY_ALLOCATION_ERROR;
                	return t;
                }
                transliterators.addElement(temprbt, status);
            }
        }

        t = new CompoundTransliterator(transliterators, passNumber - 1, parseError, status);
        // Null pointer check
        if (t != NULL) {
            t->setID(ID);
            t->adoptFilter(parser.orphanCompoundFilter());
        }
    }
    if (U_SUCCESS(status) && t == NULL) {
        status = U_MEMORY_ALLOCATION_ERROR;
    }
    return t;
}

UnicodeString& Transliterator::toRules(UnicodeString& rulesSource,
                                       UBool escapeUnprintable) const {
    // The base class implementation of toRules munges the ID into
    // the correct format.  That is: foo => ::foo
    if (escapeUnprintable) {
        rulesSource.truncate(0);
        UnicodeString id = getID();
        for (int32_t i=0; i<id.length();) {
            UChar32 c = id.char32At(i);
            if (!ICU_Utility::escapeUnprintable(rulesSource, c)) {
                rulesSource.append(c);
            }
            i += UTF_CHAR_LENGTH(c);
        }
    } else {
        rulesSource = getID();
    }
    // KEEP in sync with rbt_pars
    rulesSource.insert(0, UNICODE_STRING_SIMPLE("::"));
    rulesSource.append(ID_DELIM);
    return rulesSource;
}

int32_t Transliterator::countElements() const {
    return (this->getDynamicClassID() ==
            CompoundTransliterator::getStaticClassID()) ?
        ((const CompoundTransliterator*) this)->getCount() : 0;
}

const Transliterator& Transliterator::getElement(int32_t index, UErrorCode& ec) const {
    if (U_FAILURE(ec)) {
        return *this;
    }
    const CompoundTransliterator* cpd =
        (this->getDynamicClassID() == CompoundTransliterator::getStaticClassID()) ?
        (const CompoundTransliterator*) this : 0;
    int32_t n = (cpd == NULL) ? 1 : cpd->getCount();
    if (index < 0 || index >= n) {
        ec = U_INDEX_OUTOFBOUNDS_ERROR;
        return *this;
    } else {
        return (n == 1) ? *this : cpd->getTransliterator(index);
    }
}

UnicodeSet& Transliterator::getSourceSet(UnicodeSet& result) const {
    handleGetSourceSet(result);
    if (filter != NULL) {
        UnicodeSet* filterSet;
        UBool deleteFilterSet = FALSE;
        // Most, but not all filters will be UnicodeSets.  Optimize for
        // the high-runner case.
        if (filter->getDynamicClassID() == UnicodeSet::getStaticClassID()) {
            filterSet = (UnicodeSet*) filter;
        } else {
            filterSet = new UnicodeSet();
            // Check null pointer
            if (filterSet == NULL) {
                return result;
            }
            deleteFilterSet = TRUE;
            filter->addMatchSetTo(*filterSet);
        }
        result.retainAll(*filterSet);
        if (deleteFilterSet) {
            delete filterSet;
        }
    }
    return result;
}

void Transliterator::handleGetSourceSet(UnicodeSet& result) const {
    result.clear();
}

UnicodeSet& Transliterator::getTargetSet(UnicodeSet& result) const {
    return result.clear();
}

// For public consumption
void U_EXPORT2 Transliterator::registerFactory(const UnicodeString& id,
                                     Transliterator::Factory factory,
                                     Transliterator::Token context) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _registerFactory(id, factory, context);
    }
}

// To be called only by Transliterator subclasses that are called
// to register themselves by initializeRegistry().
void Transliterator::_registerFactory(const UnicodeString& id,
                                      Transliterator::Factory factory,
                                      Transliterator::Token context) {
    UErrorCode ec = U_ZERO_ERROR;
    registry->put(id, factory, context, TRUE, ec);
}

// To be called only by Transliterator subclasses that are called
// to register themselves by initializeRegistry().
void Transliterator::_registerSpecialInverse(const UnicodeString& target,
                                             const UnicodeString& inverseTarget,
                                             UBool bidirectional) {
    UErrorCode status = U_ZERO_ERROR;
    TransliteratorIDParser::registerSpecialInverse(target, inverseTarget, bidirectional, status);
}

void U_EXPORT2 Transliterator::registerInstance(Transliterator* adoptedPrototype) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _registerInstance(adoptedPrototype);
    }
}

void Transliterator::_registerInstance(Transliterator* adoptedPrototype) {
    UErrorCode ec = U_ZERO_ERROR;
    registry->put(adoptedPrototype, TRUE, ec);
}

void U_EXPORT2 Transliterator::registerAlias(const UnicodeString& aliasID,
                                             const UnicodeString& realID) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _registerAlias(aliasID, realID);
    }
}

void Transliterator::_registerAlias(const UnicodeString& aliasID,
                                    const UnicodeString& realID) {
    UErrorCode ec = U_ZERO_ERROR;
    registry->put(aliasID, realID, FALSE, TRUE, ec);
}

void U_EXPORT2 Transliterator::unregister(const UnicodeString& ID) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        registry->remove(ID);
    }
}

int32_t U_EXPORT2 Transliterator::countAvailableIDs(void) {
    int32_t retVal = 0;
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        retVal = registry->countAvailableIDs();
    }
    return retVal;
}

const UnicodeString& U_EXPORT2 Transliterator::getAvailableID(int32_t index) {
    const UnicodeString* result = NULL;
    umtx_lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        result = &registry->getAvailableID(index);
    }
    umtx_unlock(&registryMutex);
    U_ASSERT(result != NULL); // fail if no registry
    return *result;
}

StringEnumeration* U_EXPORT2 Transliterator::getAvailableIDs(UErrorCode& ec) {
    if (U_FAILURE(ec)) return NULL;
    StringEnumeration* result = NULL;
    umtx_lock(&registryMutex);
    if (HAVE_REGISTRY(ec)) {
        result = registry->getAvailableIDs();
    }
    umtx_unlock(&registryMutex);
    if (result == NULL) {
        ec = U_INTERNAL_TRANSLITERATOR_ERROR;
    }
    return result;
}

int32_t U_EXPORT2 Transliterator::countAvailableSources(void) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    return HAVE_REGISTRY(ec) ? _countAvailableSources() : 0;
}

UnicodeString& U_EXPORT2 Transliterator::getAvailableSource(int32_t index,
                                                  UnicodeString& result) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _getAvailableSource(index, result);
    }
    return result;
}

int32_t U_EXPORT2 Transliterator::countAvailableTargets(const UnicodeString& source) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    return HAVE_REGISTRY(ec) ? _countAvailableTargets(source) : 0;
}

UnicodeString& U_EXPORT2 Transliterator::getAvailableTarget(int32_t index,
                                                  const UnicodeString& source,
                                                  UnicodeString& result) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _getAvailableTarget(index, source, result);
    }
    return result;
}

int32_t U_EXPORT2 Transliterator::countAvailableVariants(const UnicodeString& source,
                                               const UnicodeString& target) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    return HAVE_REGISTRY(ec) ? _countAvailableVariants(source, target) : 0;
}

UnicodeString& U_EXPORT2 Transliterator::getAvailableVariant(int32_t index,
                                                   const UnicodeString& source,
                                                   const UnicodeString& target,
                                                   UnicodeString& result) {
    Mutex lock(&registryMutex);
    UErrorCode ec = U_ZERO_ERROR;
    if (HAVE_REGISTRY(ec)) {
        _getAvailableVariant(index, source, target, result);
    }
    return result;
}

int32_t Transliterator::_countAvailableSources(void) {
    return registry->countAvailableSources();
}

UnicodeString& Transliterator::_getAvailableSource(int32_t index,
                                                  UnicodeString& result) {
    return registry->getAvailableSource(index, result);
}

int32_t Transliterator::_countAvailableTargets(const UnicodeString& source) {
    return registry->countAvailableTargets(source);
}

UnicodeString& Transliterator::_getAvailableTarget(int32_t index,
                                                  const UnicodeString& source,
                                                  UnicodeString& result) {
    return registry->getAvailableTarget(index, source, result);
}

int32_t Transliterator::_countAvailableVariants(const UnicodeString& source,
                                               const UnicodeString& target) {
    return registry->countAvailableVariants(source, target);
}

UnicodeString& Transliterator::_getAvailableVariant(int32_t index,
                                                   const UnicodeString& source,
                                                   const UnicodeString& target,
                                                   UnicodeString& result) {
    return registry->getAvailableVariant(index, source, target, result);
}

#ifdef U_USE_DEPRECATED_TRANSLITERATOR_API

UChar Transliterator::filteredCharAt(const Replaceable& text, int32_t i) const {
    UChar c;
    const UnicodeFilter* localFilter = getFilter();
    return (localFilter == 0) ? text.charAt(i) :
        (localFilter->contains(c = text.charAt(i)) ? c : (UChar)0xFFFE);
}

#endif

UBool Transliterator::initializeRegistry(UErrorCode &status) {
    if (registry != 0) {
        return TRUE;
    }

    registry = new TransliteratorRegistry(status);
    if (registry == 0 || U_FAILURE(status)) {
        delete registry;
        registry = 0;
        return FALSE; // can't create registry, no recovery
    }

    /* The following code parses the index table located in
     * icu/data/translit/root.txt.  The index is an n x 4 table
     * that follows this format:
     *  <id>{
     *      file{
     *          resource{"<resource>"}
     *          direction{"<direction>"}
     *      }
     *  }
     *  <id>{
     *      internal{
     *          resource{"<resource>"}
     *          direction{"<direction"}
     *       }
     *  }
     *  <id>{
     *      alias{"<getInstanceArg"}
     *  }
     * <id> is the ID of the system transliterator being defined.  These
     * are public IDs enumerated by Transliterator.getAvailableIDs(),
     * unless the second field is "internal".
     * 
     * <resource> is a ResourceReader resource name.  Currently these refer
     * to file names under com/ibm/text/resources.  This string is passed
     * directly to ResourceReader, together with <encoding>.
     * 
     * <direction> is either "FORWARD" or "REVERSE".
     * 
     * <getInstanceArg> is a string to be passed directly to
     * Transliterator.getInstance().  The returned Transliterator object
     * then has its ID changed to <id> and is returned.
     *
     * The extra blank field on "alias" lines is to make the array square.
     */
    //static const char translit_index[] = "translit_index";

    UResourceBundle *bundle, *transIDs, *colBund;
    bundle = ures_open(U_ICUDATA_TRANSLIT, NULL/*open default locale*/, &status);
    transIDs = ures_getByKey(bundle, RB_RULE_BASED_IDS, 0, &status);

    int32_t row, maxRows;
    if (U_SUCCESS(status)) {
        maxRows = ures_getSize(transIDs);
        for (row = 0; row < maxRows; row++) {
            colBund = ures_getByIndex(transIDs, row, 0, &status);
            if (U_SUCCESS(status)) {
                UnicodeString id(ures_getKey(colBund), -1, US_INV);
                UResourceBundle* res = ures_getNextResource(colBund, NULL, &status);
                const char* typeStr = ures_getKey(res);
                UChar type;
                u_charsToUChars(typeStr, &type, 1);

                if (U_SUCCESS(status)) {
                    int32_t len = 0;
                    const UChar *resString;
                    switch (type) {
                    case 0x66: // 'f'
                    case 0x69: // 'i'
                        // 'file' or 'internal';
                        // row[2]=resource, row[3]=direction
                        {
                            
                            resString = ures_getStringByKey(res, "resource", &len, &status);
                            UBool visible = (type == 0x0066 /*f*/);
                            UTransDirection dir = 
                                (ures_getUnicodeStringByKey(res, "direction", &status).charAt(0) ==
                                 0x0046 /*F*/) ?
                                UTRANS_FORWARD : UTRANS_REVERSE;
                            registry->put(id, UnicodeString(TRUE, resString, len), dir, TRUE, visible, status);
                        }
                        break;
                    case 0x61: // 'a'
                        // 'alias'; row[2]=createInstance argument
                        resString = ures_getString(res, &len, &status);
                        registry->put(id, UnicodeString(TRUE, resString, len), TRUE, TRUE, status);
                        break;
                    }
                }
                ures_close(res);
            }
            ures_close(colBund);
        }
    }

    ures_close(transIDs);
    ures_close(bundle);

    // Manually add prototypes that the system knows about to the
    // cache.  This is how new non-rule-based transliterators are
    // added to the system.
    
    // This is to allow for null pointer check
    NullTransliterator* tempNullTranslit = new NullTransliterator();
    LowercaseTransliterator* tempLowercaseTranslit = new LowercaseTransliterator();
    UppercaseTransliterator* tempUppercaseTranslit = new UppercaseTransliterator();
    TitlecaseTransliterator* tempTitlecaseTranslit = new TitlecaseTransliterator();
    UnicodeNameTransliterator* tempUnicodeTranslit = new UnicodeNameTransliterator();
    NameUnicodeTransliterator* tempNameUnicodeTranslit = new NameUnicodeTransliterator();
#if !UCONFIG_NO_BREAK_ITERATION
     // TODO: could or should these transliterators be referenced polymorphically once constructed?
     BreakTransliterator* tempBreakTranslit         = new BreakTransliterator();
#endif
    // Check for null pointers
    if (tempNullTranslit == NULL || tempLowercaseTranslit == NULL || tempUppercaseTranslit == NULL ||
        tempTitlecaseTranslit == NULL || tempUnicodeTranslit == NULL || 
#if !UCONFIG_NO_BREAK_ITERATION
        tempBreakTranslit == NULL ||
#endif
        tempNameUnicodeTranslit == NULL )
    {
        delete tempNullTranslit;
        delete tempLowercaseTranslit;
        delete tempUppercaseTranslit;
        delete tempTitlecaseTranslit;
        delete tempUnicodeTranslit;
        delete tempNameUnicodeTranslit;
#if !UCONFIG_NO_BREAK_ITERATION
        delete tempBreakTranslit;
#endif
        // Since there was an error, remove registry
        delete registry;
        registry = NULL;

        status = U_MEMORY_ALLOCATION_ERROR;
        return 0;
    }

    registry->put(tempNullTranslit, TRUE, status);
    registry->put(tempLowercaseTranslit, TRUE, status);
    registry->put(tempUppercaseTranslit, TRUE, status);
    registry->put(tempTitlecaseTranslit, TRUE, status);
    registry->put(tempUnicodeTranslit, TRUE, status);
    registry->put(tempNameUnicodeTranslit, TRUE, status);
#if !UCONFIG_NO_BREAK_ITERATION
    registry->put(tempBreakTranslit, FALSE, status);   // FALSE means invisible.
#endif

    RemoveTransliterator::registerIDs(); // Must be within mutex
    EscapeTransliterator::registerIDs();
    UnescapeTransliterator::registerIDs();
    NormalizationTransliterator::registerIDs();
    AnyTransliterator::registerIDs();

    _registerSpecialInverse(UNICODE_STRING_SIMPLE("Null"),
                            UNICODE_STRING_SIMPLE("Null"), FALSE);
    _registerSpecialInverse(UNICODE_STRING_SIMPLE("Upper"),
                            UNICODE_STRING_SIMPLE("Lower"), TRUE);
    _registerSpecialInverse(UNICODE_STRING_SIMPLE("Title"),
                            UNICODE_STRING_SIMPLE("Lower"), FALSE);

    ucln_i18n_registerCleanup(UCLN_I18N_TRANSLITERATOR, utrans_transliterator_cleanup);

    return TRUE;
}

U_NAMESPACE_END

// Defined in ucln_in.h:

U_CFUNC UBool utrans_transliterator_cleanup(void) {
    U_NAMESPACE_USE
    TransliteratorIDParser::cleanup();
    if (registry) {
        delete registry;
        registry = NULL;
    }
    umtx_destroy(&registryMutex);
    return TRUE;
}

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

//eof
