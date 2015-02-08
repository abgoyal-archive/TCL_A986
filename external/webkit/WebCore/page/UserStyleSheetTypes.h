
 
#ifndef UserStyleSheetTypes_h
#define UserStyleSheetTypes_h

#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace WebCore {

class DOMWrapperWorld;
class UserStyleSheet;

typedef Vector<OwnPtr<UserStyleSheet> > UserStyleSheetVector;
typedef HashMap<RefPtr<DOMWrapperWorld>, UserStyleSheetVector*> UserStyleSheetMap;

} // namespace WebCore
 
#endif // UserStyleSheetTypes_h
