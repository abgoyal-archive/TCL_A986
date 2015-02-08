
#include "unicode/utypes.h"

#if !UCONFIG_NO_NORMALIZATION

#include "unicode/uniset.h"
#include "unicode/unistr.h"
#include "unicode/chariter.h"
#include "unicode/schriter.h"
#include "unicode/uchriter.h"
#include "unicode/normlzr.h"
#include "cmemory.h"
#include "normalizer2impl.h"
#include "uprops.h"  // for uniset_getUnicode32Instance()

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(Normalizer)

//-------------------------------------------------------------------------
// Constructors and other boilerplate
//-------------------------------------------------------------------------

Normalizer::Normalizer(const UnicodeString& str, UNormalizationMode mode) :
    UObject(), fFilteredNorm2(NULL), fNorm2(NULL), fUMode(mode), fOptions(0),
    text(new StringCharacterIterator(str)),
    currentIndex(0), nextIndex(0),
    buffer(), bufferPos(0)
{
    init();
}

Normalizer::Normalizer(const UChar *str, int32_t length, UNormalizationMode mode) :
    UObject(), fFilteredNorm2(NULL), fNorm2(NULL), fUMode(mode), fOptions(0),
    text(new UCharCharacterIterator(str, length)),
    currentIndex(0), nextIndex(0),
    buffer(), bufferPos(0)
{
    init();
}

Normalizer::Normalizer(const CharacterIterator& iter, UNormalizationMode mode) :
    UObject(), fFilteredNorm2(NULL), fNorm2(NULL), fUMode(mode), fOptions(0),
    text(iter.clone()),
    currentIndex(0), nextIndex(0),
    buffer(), bufferPos(0)
{
    init();
}

Normalizer::Normalizer(const Normalizer &copy) :
    UObject(copy), fFilteredNorm2(NULL), fNorm2(NULL), fUMode(copy.fUMode), fOptions(copy.fOptions),
    text(copy.text->clone()),
    currentIndex(copy.currentIndex), nextIndex(copy.nextIndex),
    buffer(copy.buffer), bufferPos(copy.bufferPos)
{
    init();
}

static const UChar _NUL=0;

void
Normalizer::init() {
    UErrorCode errorCode=U_ZERO_ERROR;
    fNorm2=Normalizer2Factory::getInstance(fUMode, errorCode);
    if(fOptions&UNORM_UNICODE_3_2) {
        delete fFilteredNorm2;
        fNorm2=fFilteredNorm2=
            new FilteredNormalizer2(*fNorm2, *uniset_getUnicode32Instance(errorCode));
    }
    if(U_FAILURE(errorCode)) {
        errorCode=U_ZERO_ERROR;
        fNorm2=Normalizer2Factory::getNoopInstance(errorCode);
    }
}

Normalizer::~Normalizer()
{
    delete fFilteredNorm2;
    delete text;
}

Normalizer* 
Normalizer::clone() const
{
    return new Normalizer(*this);
}

int32_t Normalizer::hashCode() const
{
    return text->hashCode() + fUMode + fOptions + buffer.hashCode() + bufferPos + currentIndex + nextIndex;
}
    
UBool Normalizer::operator==(const Normalizer& that) const
{
    return
        this==&that ||
        fUMode==that.fUMode &&
        fOptions==that.fOptions &&
        *text==*that.text &&
        buffer==that.buffer &&
        bufferPos==that.bufferPos &&
        nextIndex==that.nextIndex;
}

//-------------------------------------------------------------------------
// Static utility methods
//-------------------------------------------------------------------------

void U_EXPORT2
Normalizer::normalize(const UnicodeString& source, 
                      UNormalizationMode mode, int32_t options,
                      UnicodeString& result, 
                      UErrorCode &status) {
    if(source.isBogus() || U_FAILURE(status)) {
        result.setToBogus();
        if(U_SUCCESS(status)) {
            status=U_ILLEGAL_ARGUMENT_ERROR;
        }
    } else {
        UnicodeString localDest;
        UnicodeString *dest;

        if(&source!=&result) {
            dest=&result;
        } else {
            // the source and result strings are the same object, use a temporary one
            dest=&localDest;
        }
        const Normalizer2 *n2=Normalizer2Factory::getInstance(mode, status);
        if(U_SUCCESS(status)) {
            if(options&UNORM_UNICODE_3_2) {
                FilteredNormalizer2(*n2, *uniset_getUnicode32Instance(status)).
                    normalize(source, *dest, status);
            } else {
                n2->normalize(source, *dest, status);
            }
        }
        if(dest==&localDest && U_SUCCESS(status)) {
            result=*dest;
        }
    }
}

void U_EXPORT2
Normalizer::compose(const UnicodeString& source, 
                    UBool compat, int32_t options,
                    UnicodeString& result, 
                    UErrorCode &status) {
    normalize(source, compat ? UNORM_NFKC : UNORM_NFC, options, result, status);
}

void U_EXPORT2
Normalizer::decompose(const UnicodeString& source, 
                      UBool compat, int32_t options,
                      UnicodeString& result, 
                      UErrorCode &status) {
    normalize(source, compat ? UNORM_NFKD : UNORM_NFD, options, result, status);
}

UNormalizationCheckResult
Normalizer::quickCheck(const UnicodeString& source,
                       UNormalizationMode mode, int32_t options,
                       UErrorCode &status) {
    const Normalizer2 *n2=Normalizer2Factory::getInstance(mode, status);
    if(U_SUCCESS(status)) {
        if(options&UNORM_UNICODE_3_2) {
            return FilteredNormalizer2(*n2, *uniset_getUnicode32Instance(status)).
                quickCheck(source, status);
        } else {
            return n2->quickCheck(source, status);
        }
    } else {
        return UNORM_MAYBE;
    }
}

UBool
Normalizer::isNormalized(const UnicodeString& source,
                         UNormalizationMode mode, int32_t options,
                         UErrorCode &status) {
    const Normalizer2 *n2=Normalizer2Factory::getInstance(mode, status);
    if(U_SUCCESS(status)) {
        if(options&UNORM_UNICODE_3_2) {
            return FilteredNormalizer2(*n2, *uniset_getUnicode32Instance(status)).
                isNormalized(source, status);
        } else {
            return n2->isNormalized(source, status);
        }
    } else {
        return FALSE;
    }
}

UnicodeString & U_EXPORT2
Normalizer::concatenate(UnicodeString &left, UnicodeString &right,
                        UnicodeString &result,
                        UNormalizationMode mode, int32_t options,
                        UErrorCode &errorCode) {
    if(left.isBogus() || right.isBogus() || U_FAILURE(errorCode)) {
        result.setToBogus();
        if(U_SUCCESS(errorCode)) {
            errorCode=U_ILLEGAL_ARGUMENT_ERROR;
        }
    } else {
        UnicodeString localDest;
        UnicodeString *dest;

        if(&right!=&result) {
            dest=&result;
        } else {
            // the right and result strings are the same object, use a temporary one
            dest=&localDest;
        }
        *dest=left;
        const Normalizer2 *n2=Normalizer2Factory::getInstance(mode, errorCode);
        if(U_SUCCESS(errorCode)) {
            if(options&UNORM_UNICODE_3_2) {
                FilteredNormalizer2(*n2, *uniset_getUnicode32Instance(errorCode)).
                    append(*dest, right, errorCode);
            } else {
                n2->append(*dest, right, errorCode);
            }
        }
        if(dest==&localDest && U_SUCCESS(errorCode)) {
            result=*dest;
        }
    }
    return result;
}

//-------------------------------------------------------------------------
// Iteration API
//-------------------------------------------------------------------------

UChar32 Normalizer::current() {
    if(bufferPos<buffer.length() || nextNormalize()) {
        return buffer.char32At(bufferPos);
    } else {
        return DONE;
    }
}

UChar32 Normalizer::next() {
    if(bufferPos<buffer.length() ||  nextNormalize()) {
        UChar32 c=buffer.char32At(bufferPos);
        bufferPos+=UTF_CHAR_LENGTH(c);
        return c;
    } else {
        return DONE;
    }
}

UChar32 Normalizer::previous() {
    if(bufferPos>0 || previousNormalize()) {
        UChar32 c=buffer.char32At(bufferPos-1);
        bufferPos-=UTF_CHAR_LENGTH(c);
        return c;
    } else {
        return DONE;
    }
}

void Normalizer::reset() {
    currentIndex=nextIndex=text->setToStart();
    clearBuffer();
}

void
Normalizer::setIndexOnly(int32_t index) {
    text->setIndex(index);  // pins index
    currentIndex=nextIndex=text->getIndex();
    clearBuffer();
}

UChar32 Normalizer::first() {
    reset();
    return next();
}

UChar32 Normalizer::last() {
    currentIndex=nextIndex=text->setToEnd();
    clearBuffer();
    return previous();
}

int32_t Normalizer::getIndex() const {
    if(bufferPos<buffer.length()) {
        return currentIndex;
    } else {
        return nextIndex;
    }
}

int32_t Normalizer::startIndex() const {
    return text->startIndex();
}

int32_t Normalizer::endIndex() const {
    return text->endIndex();
}

//-------------------------------------------------------------------------
// Property access methods
//-------------------------------------------------------------------------

void
Normalizer::setMode(UNormalizationMode newMode) 
{
    fUMode = newMode;
    init();
}

UNormalizationMode
Normalizer::getUMode() const
{
    return fUMode;
}

void
Normalizer::setOption(int32_t option, 
                      UBool value) 
{
    if (value) {
        fOptions |= option;
    } else {
        fOptions &= (~option);
    }
    init();
}

UBool
Normalizer::getOption(int32_t option) const
{
    return (fOptions & option) != 0;
}

void
Normalizer::setText(const UnicodeString& newText, 
                    UErrorCode &status)
{
    if (U_FAILURE(status)) {
        return;
    }
    CharacterIterator *newIter = new StringCharacterIterator(newText);
    if (newIter == NULL) {
        status = U_MEMORY_ALLOCATION_ERROR;
        return;
    }
    delete text;
    text = newIter;
    reset();
}

void
Normalizer::setText(const CharacterIterator& newText, 
                    UErrorCode &status) 
{
    if (U_FAILURE(status)) {
        return;
    }
    CharacterIterator *newIter = newText.clone();
    if (newIter == NULL) {
        status = U_MEMORY_ALLOCATION_ERROR;
        return;
    }
    delete text;
    text = newIter;
    reset();
}

void
Normalizer::setText(const UChar* newText,
                    int32_t length,
                    UErrorCode &status)
{
    if (U_FAILURE(status)) {
        return;
    }
    CharacterIterator *newIter = new UCharCharacterIterator(newText, length);
    if (newIter == NULL) {
        status = U_MEMORY_ALLOCATION_ERROR;
        return;
    }
    delete text;
    text = newIter;
    reset();
}

void
Normalizer::getText(UnicodeString&  result) 
{
    text->getText(result);
}

//-------------------------------------------------------------------------
// Private utility methods
//-------------------------------------------------------------------------

void Normalizer::clearBuffer() {
    buffer.remove();
    bufferPos=0;
}

UBool
Normalizer::nextNormalize() {
    clearBuffer();
    currentIndex=nextIndex;
    text->setIndex(nextIndex);
    if(!text->hasNext()) {
        return FALSE;
    }
    // Skip at least one character so we make progress.
    UnicodeString segment(text->next32PostInc());
    while(text->hasNext()) {
        UChar32 c;
        if(fNorm2->hasBoundaryBefore(c=text->next32PostInc())) {
            text->move32(-1, CharacterIterator::kCurrent);
            break;
        }
        segment.append(c);
    }
    nextIndex=text->getIndex();
    UErrorCode errorCode=U_ZERO_ERROR;
    fNorm2->normalize(segment, buffer, errorCode);
    return U_SUCCESS(errorCode) && !buffer.isEmpty();
}

UBool
Normalizer::previousNormalize() {
    clearBuffer();
    nextIndex=currentIndex;
    text->setIndex(currentIndex);
    if(!text->hasPrevious()) {
        return FALSE;
    }
    UnicodeString segment;
    while(text->hasPrevious()) {
        UChar32 c=text->previous32();
        segment.insert(0, c);
        if(fNorm2->hasBoundaryBefore(c)) {
            break;
        }
    }
    currentIndex=text->getIndex();
    UErrorCode errorCode=U_ZERO_ERROR;
    fNorm2->normalize(segment, buffer, errorCode);
    bufferPos=buffer.length();
    return U_SUCCESS(errorCode) && !buffer.isEmpty();
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_NORMALIZATION */
