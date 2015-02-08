
 
#ifndef UserScriptTypes_h
#define UserScriptTypes_h

#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace WebCore {

enum UserScriptInjectionTime { InjectAtDocumentStart, InjectAtDocumentEnd };

class DOMWrapperWorld;
class UserScript;

typedef Vector<OwnPtr<UserScript> > UserScriptVector;
typedef HashMap<RefPtr<DOMWrapperWorld>, UserScriptVector*> UserScriptMap;

} // namespace WebCore
 
#endif // UserScriptTypes_h
