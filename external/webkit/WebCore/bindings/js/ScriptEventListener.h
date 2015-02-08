

#ifndef ScriptEventListener_h
#define ScriptEventListener_h

#include "JSLazyEventListener.h"
#include "ScriptState.h"

#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Attribute;
    class EventListener;
    class Frame;
    class Node;

    PassRefPtr<JSLazyEventListener> createAttributeEventListener(Node*, Attribute*);
    PassRefPtr<JSLazyEventListener> createAttributeEventListener(Frame*, Attribute*);
    String getEventListenerHandlerBody(ScriptExecutionContext*, ScriptState*, EventListener*);

} // namespace WebCore

#endif // ScriptEventListener_h
