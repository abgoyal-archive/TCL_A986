

#ifndef V8NPUtils_h
#define V8NPUtils_h

#if PLATFORM(CHROMIUM)
#include <bindings/npruntime.h>
#else
#include "npruntime.h"
#endif

#include <v8.h>

// Convert a V8 Value of any type (string, bool, object, etc) to a NPVariant.
void convertV8ObjectToNPVariant(v8::Local<v8::Value>, NPObject*, NPVariant*);

// Convert a NPVariant (string, bool, object, etc) back to a V8 Value.  The owner object is the NPObject which relates to the
// object, if the object is an Object.  The created NPObject will be tied to the lifetime of the owner.
v8::Handle<v8::Value> convertNPVariantToV8Object(const NPVariant*, NPObject*);

// Helper function to create an NPN String Identifier from a v8 string.
NPIdentifier getStringIdentifier(v8::Handle<v8::String>);

#endif // V8NPUtils_h
