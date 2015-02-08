
#include "unicode/utypes.h"
#include "unicode/resbund.h"
#include "umutex.h"

#include "uresimp.h"

U_NAMESPACE_BEGIN

//-----------------------------------------------------------------------------

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(ResourceBundle)

ResourceBundle::ResourceBundle(UErrorCode &err)
                                :UObject(), fLocale(NULL)
{
    fResource = ures_open(0, Locale::getDefault().getName(), &err);
}

ResourceBundle::ResourceBundle(const ResourceBundle &other)
                              :UObject(other), fLocale(NULL)
{
    UErrorCode status = U_ZERO_ERROR;

    if (other.fResource) {
        fResource = ures_copyResb(0, other.fResource, &status);
    } else {
        /* Copying a bad resource bundle */
        fResource = NULL;
    }
}

ResourceBundle::ResourceBundle(UResourceBundle *res, UErrorCode& err)
                               :UObject(), fLocale(NULL)
{
    if (res) {
        fResource = ures_copyResb(0, res, &err);
    } else {
        /* Copying a bad resource bundle */
        fResource = NULL;
    }
}

ResourceBundle::ResourceBundle(const char* path, const Locale& locale, UErrorCode& err) 
                               :UObject(), fLocale(NULL)
{
    fResource = ures_open(path, locale.getName(), &err);
}


ResourceBundle& ResourceBundle::operator=(const ResourceBundle& other)
{
    if(this == &other) {
        return *this;
    }
    if(fResource != 0) {
        ures_close(fResource);
        fResource = NULL;
    }
    UErrorCode status = U_ZERO_ERROR;
    if (other.fResource) {
        fResource = ures_copyResb(0, other.fResource, &status);
    } else {
        /* Copying a bad resource bundle */
        fResource = NULL;
    }
    return *this;
}

ResourceBundle::~ResourceBundle()
{
    if(fResource != 0) {
        ures_close(fResource);
    }
    if(fLocale != NULL) {
      delete(fLocale);
    }
}

ResourceBundle *
ResourceBundle::clone() const {
    return new ResourceBundle(*this);
}

UnicodeString ResourceBundle::getString(UErrorCode& status) const {
    int32_t len = 0;
    const UChar *r = ures_getString(fResource, &len, &status);
    return UnicodeString(TRUE, r, len);
}

const uint8_t *ResourceBundle::getBinary(int32_t& len, UErrorCode& status) const {
    return ures_getBinary(fResource, &len, &status);
}

const int32_t *ResourceBundle::getIntVector(int32_t& len, UErrorCode& status) const {
    return ures_getIntVector(fResource, &len, &status);
}

uint32_t ResourceBundle::getUInt(UErrorCode& status) const {
    return ures_getUInt(fResource, &status);
}

int32_t ResourceBundle::getInt(UErrorCode& status) const {
    return ures_getInt(fResource, &status);
}

const char *ResourceBundle::getName(void) const {
    return ures_getName(fResource);
}

const char *ResourceBundle::getKey(void) const {
    return ures_getKey(fResource);
}

UResType ResourceBundle::getType(void) const {
    return ures_getType(fResource);
}

int32_t ResourceBundle::getSize(void) const {
    return ures_getSize(fResource);
}

UBool ResourceBundle::hasNext(void) const {
    return ures_hasNext(fResource);
}

void ResourceBundle::resetIterator(void) {
    ures_resetIterator(fResource);
}

ResourceBundle ResourceBundle::getNext(UErrorCode& status) {
    UResourceBundle r;

    ures_initStackObject(&r);
    ures_getNextResource(fResource, &r, &status);
    ResourceBundle res(&r, status);
    if (U_SUCCESS(status)) {
        ures_close(&r);
    }
    return res;
}

UnicodeString ResourceBundle::getNextString(UErrorCode& status) {
    int32_t len = 0;
    const UChar* r = ures_getNextString(fResource, &len, 0, &status);
    return UnicodeString(TRUE, r, len);
}

UnicodeString ResourceBundle::getNextString(const char ** key, UErrorCode& status) {
    int32_t len = 0;
    const UChar* r = ures_getNextString(fResource, &len, key, &status);
    return UnicodeString(TRUE, r, len);
}

ResourceBundle ResourceBundle::get(int32_t indexR, UErrorCode& status) const {
    UResourceBundle r;

    ures_initStackObject(&r);
    ures_getByIndex(fResource, indexR, &r, &status);
    ResourceBundle res(&r, status);
    if (U_SUCCESS(status)) {
        ures_close(&r);
    }
    return res;
}

UnicodeString ResourceBundle::getStringEx(int32_t indexS, UErrorCode& status) const {
    int32_t len = 0;
    const UChar* r = ures_getStringByIndex(fResource, indexS, &len, &status);
    return UnicodeString(TRUE, r, len);
}

ResourceBundle ResourceBundle::get(const char* key, UErrorCode& status) const {
    UResourceBundle r;

    ures_initStackObject(&r);
    ures_getByKey(fResource, key, &r, &status);
    ResourceBundle res(&r, status);
    if (U_SUCCESS(status)) {
        ures_close(&r);
    }
    return res;
}

ResourceBundle ResourceBundle::getWithFallback(const char* key, UErrorCode& status){
    UResourceBundle r;
    ures_initStackObject(&r);
    ures_getByKeyWithFallback(fResource, key, &r, &status);
    ResourceBundle res(&r, status);
    if(U_SUCCESS(status)){
        ures_close(&r);
    }
    return res;
}
UnicodeString ResourceBundle::getStringEx(const char* key, UErrorCode& status) const {
    int32_t len = 0;
    const UChar* r = ures_getStringByKey(fResource, key, &len, &status);
    return UnicodeString(TRUE, r, len);
}

const char*
ResourceBundle::getVersionNumber()  const
{
    return ures_getVersionNumber(fResource);
}

void ResourceBundle::getVersion(UVersionInfo versionInfo) const {
    ures_getVersion(fResource, versionInfo);
}

const Locale &ResourceBundle::getLocale(void) const
{
    UBool needInit;
    UMTX_CHECK(NULL, (fLocale == NULL), needInit);
    if(needInit) {
        UErrorCode status = U_ZERO_ERROR;
        const char *localeName = ures_getLocale(fResource, &status);
        Locale  *tLocale = new Locale(localeName);
        // Null pointer check
        if (tLocale == NULL) {
        	return Locale::getDefault(); // Return default locale if one could not be created.
        }
        umtx_lock(NULL);
        ResourceBundle *me = (ResourceBundle *)this; // semantically const
        if (me->fLocale == NULL) {
            me->fLocale = tLocale;
            tLocale = NULL;
        }
        umtx_unlock(NULL);
        delete tLocale;
    }
    return *fLocale;
}

const Locale ResourceBundle::getLocale(ULocDataLocaleType type, UErrorCode &status) const
{
  return ures_getLocaleByType(fResource, type, &status);
}

//eof
U_NAMESPACE_END
