

#ifndef SoftLinking_h
#define SoftLinking_h

#include <windows.h>
#include <wtf/Assertions.h>

#define SOFT_LINK_LIBRARY_HELPER(lib, suffix) \
    static HMODULE lib##Library() \
    { \
        static HMODULE library = LoadLibraryW(L###lib suffix); \
        ASSERT(library); \
        return library; \
    }

#define SOFT_LINK_LIBRARY(lib) SOFT_LINK_LIBRARY_HELPER(lib, L".dll")
#define SOFT_LINK_DEBUG_LIBRARY(lib) SOFT_LINK_LIBRARY_HELPER(lib, L"_debug.dll")

#define SOFT_LINK(library, functionName, resultType, callingConvention, parameterDeclarations, parameterNames) \
    static resultType callingConvention init##functionName parameterDeclarations; \
    static resultType (callingConvention*softLink##functionName) parameterDeclarations = init##functionName; \
    \
    static resultType callingConvention init##functionName parameterDeclarations \
    { \
        softLink##functionName = reinterpret_cast<resultType (callingConvention*) parameterDeclarations>(GetProcAddress(library##Library(), #functionName)); \
        ASSERT(softLink##functionName); \
        return softLink##functionName parameterNames; \
    }\
    \
    inline resultType functionName parameterDeclarations \
    {\
        return softLink##functionName parameterNames; \
    }

#define SOFT_LINK_OPTIONAL(library, functionName, resultType, callingConvention, parameterDeclarations) \
    typedef resultType (callingConvention *functionName##PtrType) parameterDeclarations; \
    static functionName##PtrType functionName##Ptr() \
    { \
        static functionName##PtrType ptr; \
        static bool initialized; \
        \
        if (initialized) \
            return ptr; \
        initialized = true; \
        \
        ptr = reinterpret_cast<functionName##PtrType>(GetProcAddress(library##Library(), #functionName)); \
        return ptr; \
    }\

#endif // SoftLinking_h
