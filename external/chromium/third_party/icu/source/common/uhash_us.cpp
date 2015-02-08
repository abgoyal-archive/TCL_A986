
#include "uhash.h"
#include "hash.h"
#include "uvector.h"
#include "unicode/unistr.h"
#include "unicode/uchar.h"


U_CAPI int32_t U_EXPORT2
uhash_hashUnicodeString(const UHashTok key) {
    U_NAMESPACE_USE
    const UnicodeString *str = (const UnicodeString*) key.pointer;
    return (str == NULL) ? 0 : str->hashCode();
}

U_CAPI void U_EXPORT2
uhash_deleteUnicodeString(void *obj) {
    U_NAMESPACE_USE
    delete (UnicodeString*) obj;
}

U_CAPI UBool U_EXPORT2
uhash_compareUnicodeString(const UHashTok key1, const UHashTok key2) {
    U_NAMESPACE_USE
    const UnicodeString *str1 = (const UnicodeString*) key1.pointer;
    const UnicodeString *str2 = (const UnicodeString*) key2.pointer;
    if (str1 == str2) {
        return TRUE;
    }
    if (str1 == NULL || str2 == NULL) {
        return FALSE;
    }
    return *str1 == *str2;
}

U_CAPI void U_EXPORT2
uhash_deleteHashtable(void *obj) {
    U_NAMESPACE_USE
    delete (Hashtable*) obj;
}

U_CAPI void U_EXPORT2
uhash_deleteUVector(void *obj) {
    U_NAMESPACE_USE
    delete (UVector*) obj;
}

//eof
