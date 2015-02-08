

#import <wtf/Assertions.h>
#import <dlfcn.h>

#define SOFT_LINK_LIBRARY(lib) \
    static void* lib##Library() \
    { \
        static void* dylib = dlopen("/usr/lib/" #lib ".dylib", RTLD_NOW); \
        ASSERT(dylib); \
        return dylib; \
    }

#define SOFT_LINK_FRAMEWORK(framework) \
    static void* framework##Library() \
    { \
        static void* frameworkLibrary = dlopen("/System/Library/Frameworks/" #framework ".framework/" #framework, RTLD_NOW); \
        ASSERT(frameworkLibrary); \
        return frameworkLibrary; \
    }

#define SOFT_LINK(framework, functionName, resultType, parameterDeclarations, parameterNames) \
    static resultType init##functionName parameterDeclarations; \
    static resultType (*softLink##functionName) parameterDeclarations = init##functionName; \
    \
    static resultType init##functionName parameterDeclarations \
    { \
        softLink##functionName = (resultType (*) parameterDeclarations) dlsym(framework##Library(), #functionName); \
        ASSERT(softLink##functionName); \
        return softLink##functionName parameterNames; \
    }\
    \
    inline resultType functionName parameterDeclarations \
    {\
        return softLink##functionName parameterNames; \
    }

#define SOFT_LINK_CLASS(framework, className) \
    static Class init##className(); \
    static Class (*get##className##Class)() = init##className; \
    static Class class##className; \
    \
    static Class className##Function() \
    { \
        return class##className; \
    }\
    \
    static Class init##className() \
    { \
        framework##Library(); \
        class##className = objc_getClass(#className); \
        ASSERT(class##className); \
        get##className##Class = className##Function; \
        return class##className; \
    }

#define SOFT_LINK_POINTER(framework, name, type) \
    static type init##name(); \
    static type (*get##name)() = init##name; \
    static type pointer##name; \
    \
    static type name##Function() \
    { \
        return pointer##name; \
    }\
    \
    static type init##name() \
    { \
        void** pointer = static_cast<void**>(dlsym(framework##Library(), #name)); \
        ASSERT(pointer); \
        pointer##name = static_cast<type>(*pointer); \
        get##name = name##Function; \
        return pointer##name; \
    }

#define SOFT_LINK_CONSTANT(framework, name, type) \
    static type init##name(); \
    static type (*get##name)() = init##name; \
    static type constant##name; \
    \
    static type name##Function() \
    { \
        return constant##name; \
    }\
    \
    static type init##name() \
    { \
        void* constant = dlsym(framework##Library(), #name); \
        ASSERT(constant); \
        constant##name = *static_cast<type*>(constant); \
        get##name = name##Function; \
        return constant##name; \
    }
