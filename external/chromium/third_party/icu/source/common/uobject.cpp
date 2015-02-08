
#include "unicode/uobject.h"
#include "cmemory.h"

U_NAMESPACE_BEGIN

#if U_OVERRIDE_CXX_ALLOCATION


void * U_EXPORT2 UMemory::operator new(size_t size) U_NO_THROW {
    return uprv_malloc(size);
}

void U_EXPORT2 UMemory::operator delete(void *p) U_NO_THROW {
    if(p!=NULL) {
        uprv_free(p);
    }
}

void * U_EXPORT2 UMemory::operator new[](size_t size) U_NO_THROW {
    return uprv_malloc(size);
}

void U_EXPORT2 UMemory::operator delete[](void *p) U_NO_THROW {
    if(p!=NULL) {
        uprv_free(p);
    }
}

#if U_HAVE_DEBUG_LOCATION_NEW
void * U_EXPORT2 UMemory::operator new(size_t size, const char* /*file*/, int /*line*/) U_NO_THROW {
    return UMemory::operator new(size);
}

void U_EXPORT2 UMemory::operator delete(void* p, const char* /*file*/, int /*line*/) U_NO_THROW {
    UMemory::operator delete(p);
}
#endif /* U_HAVE_DEBUG_LOCATION_NEW */


#endif

UObject::~UObject() {}

// Future implementation for RTTI that support subtyping. [alan]
// 
// UClassID UObject::getStaticClassID() {
//     return (UClassID) NULL;
// }
// 
// UBool UObject::instanceOf(UClassID type) const {
//     UClassID c = getDynamicClassID();
//     for (;;) {
//         if (c == type) {
//             return TRUE;
//         } else if (c == (UClassID) NULL) {
//             return FALSE;
//         }
//         c = * (UClassID*) c;
//     }
// }

U_NAMESPACE_END


