

#ifndef PopStateEvent_h
#define PopStateEvent_h

#include "Event.h"
#include "SerializedScriptValue.h"

namespace WebCore {

class SerializedScriptValue;

class PopStateEvent : public Event {
public:
    static PassRefPtr<PopStateEvent> create(PassRefPtr<SerializedScriptValue> stateObject)
    {
        return adoptRef(new PopStateEvent(stateObject));
    }

    void initPopStateEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue>);
    bool isPopStateEvent() const { return true; }

    SerializedScriptValue* state() const { return m_stateObject.get(); }    

private:
    PopStateEvent(PassRefPtr<SerializedScriptValue>);

    RefPtr<SerializedScriptValue> m_stateObject;
};

} // namespace WebCore

#endif // PopStateEvent_h
