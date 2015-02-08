

#ifndef DOMApplicationCache_h
#define DOMApplicationCache_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "ApplicationCacheHost.h"
#include "AtomicStringHash.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

class AtomicStringImpl;
class Frame;
class KURL;
class String;

class DOMApplicationCache : public RefCounted<DOMApplicationCache>, public EventTarget {
public:
    static PassRefPtr<DOMApplicationCache> create(Frame* frame) { return adoptRef(new DOMApplicationCache(frame)); }
    ~DOMApplicationCache() { ASSERT(!m_frame); }

    void disconnectFrame();

    unsigned short status() const;
    void update(ExceptionCode&);
    void swapCache(ExceptionCode&);

    // EventTarget impl

    using RefCounted<DOMApplicationCache>::ref;
    using RefCounted<DOMApplicationCache>::deref;

    // Explicitly named attribute event listener helpers

    DEFINE_ATTRIBUTE_EVENT_LISTENER(checking);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(error);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(noupdate);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(downloading);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(progress);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(updateready);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(cached);
    DEFINE_ATTRIBUTE_EVENT_LISTENER(obsolete);

    virtual ScriptExecutionContext* scriptExecutionContext() const;
    DOMApplicationCache* toDOMApplicationCache() { return this; }

    static const AtomicString& toEventType(ApplicationCacheHost::EventID id);

private:
    DOMApplicationCache(Frame*);

    virtual void refEventTarget() { ref(); }
    virtual void derefEventTarget() { deref(); }
    virtual EventTargetData* eventTargetData();
    virtual EventTargetData* ensureEventTargetData();

    ApplicationCacheHost* applicationCacheHost() const;

    Frame* m_frame;
    EventTargetData m_eventTargetData;
};

} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)

#endif // DOMApplicationCache_h
