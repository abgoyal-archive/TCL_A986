

#ifndef JSWebSocketConstructor_h
#define JSWebSocketConstructor_h

#if ENABLE(WEB_SOCKETS)

#include "JSDOMBinding.h"

namespace WebCore {

class JSWebSocketConstructor : public DOMConstructorObject {
  public:
    JSWebSocketConstructor(JSC::ExecState*, JSDOMGlobalObject*);
    static const JSC::ClassInfo s_info;

  private:
    virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
    virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
};

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // JSWebSocketConstructor_h
