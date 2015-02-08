
#ifndef IDBRequest_h
#define IDBRequest_h

#include "ActiveDOMObject.h"
#include "EventTarget.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#if ENABLE(INDEXED_DATABASE)

namespace WebCore {

class IDBDatabaseError;
class SerializedScriptValue;

class IDBRequest : public RefCounted<IDBRequest>, public ActiveDOMObject, public EventTarget {
public:
    static PassRefPtr<IDBRequest> create(ScriptExecutionContext* context)
    {
        return adoptRef(new IDBRequest(context));
    }
    ~IDBRequest();

    void abort();
    unsigned short readyState() const { return m_readyState; }
    IDBDatabaseError* error() const { return m_error.get(); }
    SerializedScriptValue* result() const { return m_result.get(); }

    DEFINE_ATTRIBUTE_EVENT_LISTENER(success);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(error);

    using RefCounted<IDBRequest>::ref;
    using RefCounted<IDBRequest>::deref;

    // EventTarget interface
    virtual ScriptExecutionContext* scriptExecutionContext() const { return ActiveDOMObject::scriptExecutionContext(); }
    virtual IDBRequest* toIDBRequest() { return this; }

private:
    explicit IDBRequest(ScriptExecutionContext* context);

    // EventTarget interface
    virtual void refEventTarget() { ref(); }
    virtual void derefEventTarget() { deref(); }
    virtual EventTargetData* eventTargetData();
    virtual EventTargetData* ensureEventTargetData();

    unsigned short m_readyState;
    RefPtr<IDBDatabaseError> m_error;
    RefPtr<SerializedScriptValue> m_result;

    EventTargetData m_eventTargetData;
};

} // namespace WebCore

#endif

#endif // IDBRequest_h

