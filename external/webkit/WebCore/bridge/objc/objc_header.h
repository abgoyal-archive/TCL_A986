

#ifndef KJS_BINDINGS_OBJC_HEADER_H
#define KJS_BINDINGS_OBJC_HEADER_H

#ifdef __OBJC__

#include <objc/objc.h>
#include <objc/objc-class.h>
#include <objc/objc-runtime.h>

typedef Class ClassStructPtr;
typedef id ObjectStructPtr;
typedef Method MethodStructPtr;
typedef Ivar IvarStructPtr;

@class NSMethodSignature;

#else

typedef struct objc_class* ClassStructPtr;
typedef struct objc_object* ObjectStructPtr;
typedef struct objc_method* MethodStructPtr;
typedef struct objc_ivar* IvarStructPtr;

class NSMethodSignature;

#endif

#endif
