

#ifndef ScriptEventListener_h
#define ScriptEventListener_h

#include "V8LazyEventListener.h"

#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Attribute;
    class EventListener;
    class Frame;
    class Node;
    class ScriptState;

    PassRefPtr<V8LazyEventListener> createAttributeEventListener(Node*, Attribute*);
    PassRefPtr<V8LazyEventListener> createAttributeEventListener(Frame*, Attribute*);
    String getEventListenerHandlerBody(ScriptExecutionContext*, ScriptState*, EventListener*);

} // namespace WebCore

#endif // ScriptEventListener_h
