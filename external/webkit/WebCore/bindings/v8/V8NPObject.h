

#ifndef V8NPObject_h
#define V8NPObject_h

#if PLATFORM(CHROMIUM)
#include <bindings/npruntime.h>
#else
#include "npruntime.h"
#endif

#include <v8.h>

// These functions can be replaced by normal JS operation.
// Getters
v8::Handle<v8::Value> npObjectNamedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo&);
v8::Handle<v8::Value> npObjectIndexedPropertyGetter(uint32_t index, const v8::AccessorInfo&);
v8::Handle<v8::Value> npObjectGetNamedProperty(v8::Local<v8::Object> self, v8::Local<v8::String> name);
v8::Handle<v8::Value> npObjectGetIndexedProperty(v8::Local<v8::Object> self, uint32_t index);

// Setters
v8::Handle<v8::Value> npObjectNamedPropertySetter(v8::Local<v8::String> name, v8::Local<v8::Value>, const v8::AccessorInfo&);
v8::Handle<v8::Value> npObjectIndexedPropertySetter(uint32_t index, const v8::AccessorInfo&);
v8::Handle<v8::Value> npObjectSetNamedProperty(v8::Local<v8::Object> self, v8::Local<v8::String> name, v8::Local<v8::Value>);
v8::Handle<v8::Value> npObjectSetIndexedProperty(v8::Local<v8::Object> self, uint32_t index, v8::Local<v8::Value>);

v8::Handle<v8::Value> npObjectInvokeDefaultHandler(const v8::Arguments&);

// Get a wrapper for a NPObject.
// If the object is already wrapped, the pre-existing wrapper will be returned. If the object is not wrapped, wrap it, and
// give V8 a weak reference to the wrapper which will cleanup when there are no more JS references to the object.
v8::Local<v8::Object> createV8ObjectForNPObject(NPObject*, NPObject* root);

// Tell V8 to forcibly remove an object.
// This is used at plugin teardown so that the caller can aggressively unload the plugin library. After calling this
// function, the persistent handle to the wrapper will be gone, and the wrapped NPObject will be removed so that it
// cannot be referred to.
void forgetV8ObjectForNPObject(NPObject*);

#endif // V8NPObject_h
