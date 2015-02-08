

#ifndef npruntime_priv_h
#define npruntime_priv_h

#if PLATFORM(CHROMIUM)
#include <bindings/npruntime.h>
#else
#include "npruntime.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void _NPN_InitializeVariantWithStringCopy(NPVariant*, const NPString*);
void _NPN_DeallocateObject(NPObject*);

// The following routines allow the browser to aggressively cleanup NPObjects
// on a per plugin basis.  All NPObjects used through the NPRuntime API should
// be "registered" while they are alive.  After an object has been
// deleted, it is possible for Javascript to have a reference to that object
// which has not yet been garbage collected.  Javascript access to NPObjects
// will reference this registry to determine if the object is accessible or
// not.

// Windows introduces an additional complication for objects created by the
// plugin.  Plugins load inside of a DLL.  Each DLL has it's own heap.  If
// the browser unloads the plugin DLL, all objects created within the DLL's
// heap instantly become invalid.  Normally, when WebKit drops the reference
// on the top-level plugin object, it tells the plugin manager that the
// plugin can be destroyed, which can unload the DLL.  So, we must eliminate
// all pointers to any object ever created by the plugin.

// We generally associate NPObjects with an owner.  The owner of an NPObject
// is an NPObject which, when destroyed, also destroys all objects it owns.
// For example, if an NPAPI plugin creates 10 sub-NPObjects, all 11 objects
// (the NPAPI plugin + its 10 sub-objects) should become inaccessible
// simultaneously.

// The ownership hierarchy is flat, and not a tree.  Imagine the following
// object creation:
//     PluginObject
//          |
//          +-- Creates -----> Object1
//                                |
//                                +-- Creates -----> Object2
//
// PluginObject will be the "owner" for both Object1 and Object2.

// Register an NPObject with the runtime.  If the owner is NULL, the
// object is treated as an owning object.  If owner is not NULL,
// this object will be registered as owned by owner's top-level owner.
void _NPN_RegisterObject(NPObject*, NPObject* owner);

// Unregister an NPObject with the runtime.  If obj is an owning
// object, this call will also unregister all of the owned objects.
void _NPN_UnregisterObject(NPObject*);

// Check to see if an object is registered with the runtime.
// Return true if registered, false otherwise.
bool _NPN_IsAlive(NPObject*);

#ifdef __cplusplus
}  /* end extern "C" */
#endif

#endif // npruntime_priv_h
