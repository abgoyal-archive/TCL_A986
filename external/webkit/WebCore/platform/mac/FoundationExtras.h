

#import <CoreFoundation/CFBase.h>
#import <Foundation/NSObject.h>

// nil-checked CFRetain/CFRelease covers for Objective-C ids

// Use CFRetain, CFRelease, HardRetain, or HardRelease instead of
// -[NSObject retain] and -[NSObject release] if you want to store
// a pointer to an Objective-C object into memory that won't
// be scanned for GC, like a C++ object.

static inline id HardRetain(id obj)
{
    if (obj) CFRetain(obj);
    return obj;
}

static inline void HardRelease(id obj)
{
    if (obj) CFRelease(obj);
}

// As if CF and Foundation had logically separate reference counts,
// this function first increments the CF retain count, and then
// decrements the NS retain count. This is needed to handle cases where
// -retain/-release aren't equivalent to CFRetain/HardRelease, such as
// when GC is used.

// Use HardRetainWithNSRelease after allocating and initializing a NSObject
// if you want to store a pointer to that object into memory that won't
// be scanned for GC, like a C++ object.

static inline id HardRetainWithNSRelease(id obj)
{
    HardRetain(obj);
    [obj release];
    return obj;
}

// Use HardAutorelease to return an object made by a CoreFoundation
// "create" or "copy" function as an autoreleased and garbage collected
// object. CF objects need to be "made collectable" for autorelease to work
// properly under GC.

static inline id HardAutorelease(CFTypeRef obj)
{
    if (obj)
        CFMakeCollectable(obj);
    [(id)obj autorelease];
    return (id)obj;
}
