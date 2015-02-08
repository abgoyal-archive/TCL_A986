

#ifdef __cplusplus
#define NULL __null
#else
#define NULL ((void *)0)
#endif

#import <stddef.h>
#import <stdio.h>
#import <fcntl.h>
#import <errno.h>
#import <unistd.h>
#import <signal.h>
#import <sys/types.h>
#import <sys/time.h>
#import <sys/resource.h>

#import <pthread.h>

#import <CoreGraphics/CoreGraphics.h>

#ifdef __cplusplus

#include <algorithm> // needed for exception_defines.h
#include <cstddef>
#include <new>

#endif

#import <ApplicationServices/ApplicationServices.h>
#import <Carbon/Carbon.h>

#ifndef CGFLOAT_DEFINED
#ifdef __LP64__
typedef double CGFloat;
#else
typedef float CGFloat;
#endif
#define CGFLOAT_DEFINED 1
#endif

#ifdef __OBJC__
#import <Cocoa/Cocoa.h>
#endif

#include <wtf/Platform.h>

#include "EmptyProtocolDefinitions.h"

/* WebKit has no way to pull settings from WebCore/config.h for now */
/* so we assume WebKit is always being compiled on top of JavaScriptCore */
#define WTF_USE_JSC 1
#define WTF_USE_V8 0

#ifdef __cplusplus
#include <wtf/FastMalloc.h>
#endif

#include <wtf/DisallowCType.h>

/* Work around bug with C++ library that screws up Objective-C++ when exception support is disabled. */
#undef try
#undef catch

#define JS_EXPORTDATA
#define JS_EXPORTCLASS
#define WEBKIT_EXPORTDATA
