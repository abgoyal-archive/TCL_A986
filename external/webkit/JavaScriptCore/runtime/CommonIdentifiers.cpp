

#include "config.h"
#include "CommonIdentifiers.h"

namespace JSC {

static const char* const nullCString = 0;

#define INITIALIZE_PROPERTY_NAME(name) , name(globalData, #name)

CommonIdentifiers::CommonIdentifiers(JSGlobalData* globalData)
    : emptyIdentifier(globalData, "")
    , underscoreProto(globalData, "__proto__")
    , thisIdentifier(globalData, "this")
    JSC_COMMON_IDENTIFIERS_EACH_PROPERTY_NAME(INITIALIZE_PROPERTY_NAME)
{
}

} // namespace JSC
