

#include "config.h"
#include "IdentifierRep.h"

#include "PlatformString.h"
#include <runtime/UString.h>
#include <wtf/HashMap.h>
#include <wtf/StdLibExtras.h>

using namespace JSC;

namespace WebCore {

typedef HashSet<IdentifierRep*> IdentifierSet;

static IdentifierSet& identifierSet()
{
    DEFINE_STATIC_LOCAL(IdentifierSet, identifierSet, ());
    return identifierSet;
}
    
typedef HashMap<int, IdentifierRep*> IntIdentifierMap;

static IntIdentifierMap& intIdentifierMap()
{
    DEFINE_STATIC_LOCAL(IntIdentifierMap, intIdentifierMap, ());
    return intIdentifierMap;
}

IdentifierRep* IdentifierRep::get(int intID)
{
    if (intID == 0 || intID == -1) {
        static IdentifierRep* negativeOneAndZeroIdentifiers[2];

        IdentifierRep* identifier = negativeOneAndZeroIdentifiers[intID + 1];
        if (!identifier) {
            identifier = new IdentifierRep(intID);

            negativeOneAndZeroIdentifiers[intID + 1] = identifier;
        }
        
        return identifier;
    }
    
    pair<IntIdentifierMap::iterator, bool> result = intIdentifierMap().add(intID, 0); 
    if (result.second) {
        ASSERT(!result.first->second);
        result.first->second = new IdentifierRep(intID);
        
        identifierSet().add(result.first->second);
    }
    
    return result.first->second;
}

typedef HashMap<RefPtr<JSC::UString::Rep>, IdentifierRep*> StringIdentifierMap;

static StringIdentifierMap& stringIdentifierMap()
{
    DEFINE_STATIC_LOCAL(StringIdentifierMap, stringIdentifierMap, ());
    return stringIdentifierMap;
}

IdentifierRep* IdentifierRep::get(const char* name)
{
    ASSERT(name);
    if (!name)
        return 0;
  
    UString string = String::fromUTF8WithLatin1Fallback(name, strlen(name));
    pair<StringIdentifierMap::iterator, bool> result = stringIdentifierMap().add(string.rep(), 0);
    if (result.second) {
        ASSERT(!result.first->second);
        result.first->second = new IdentifierRep(name);
        
        identifierSet().add(result.first->second);
    }
    
    return result.first->second;
}

bool IdentifierRep::isValid(IdentifierRep* identifier)
{
    return identifierSet().contains(identifier);
}
    
} // namespace WebCore
