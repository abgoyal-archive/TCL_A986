

#ifndef JNIUtilityPrivate_h
#define JNIUtilityPrivate_h

#include "JNIUtility.h"
#include "npruntime.h"

// FIXME: While fully implementing the bindings I noticed some differences between what
// I wrote and seemed intuitive and what JSC does. Need to verify if my intuition is wrong
// or there are bugs in the JSC bindings. For now, this macro makes the V8 bindings do the
// same as the JSC bindings.
#define EMULATE_JSC_BINDINGS 1

namespace JSC {

namespace Bindings {

jvalue convertNPVariantToJValue(NPVariant, JNIType, const char* javaClassName);
void convertJValueToNPVariant(jvalue, JNIType, const char* javaClassName, NPVariant*);

} // namespace Bindings

} // namespace JSC

#endif // JNIUtilityPrivate_h
