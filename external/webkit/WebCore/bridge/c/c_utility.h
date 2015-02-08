

#ifndef C_UTILITY_H_
#define C_UTILITY_H_

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "npruntime_internal.h"
#include <runtime/JSValue.h>

namespace WebCore {
    class String;
}

namespace JSC {

class ExecState;
class Identifier;

namespace Bindings {

class RootObject;
    
typedef uint16_t NPUTF16;

WebCore::String convertNPStringToUTF16(const NPString *string);
void convertValueToNPVariant(ExecState*, JSValue, NPVariant* result);
JSValue convertNPVariantToValue(ExecState*, const NPVariant*, RootObject*);
Identifier identifierFromNPIdentifier(const NPUTF8* name);

} }

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif
