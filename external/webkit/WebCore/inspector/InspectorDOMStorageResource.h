

#ifndef InspectorDOMStorageResource_h
#define InspectorDOMStorageResource_h

#if ENABLE(DOM_STORAGE)

#include "EventListener.h"
#include "ScriptObject.h"
#include "ScriptState.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class Storage;
    class Frame;
    class InspectorFrontend;

    class InspectorDOMStorageResource : public EventListener {
    public:
        static PassRefPtr<InspectorDOMStorageResource> create(Storage* domStorage, bool isLocalStorage, Frame* frame)
        {
            return adoptRef(new InspectorDOMStorageResource(domStorage, isLocalStorage, frame));
        }
        static const InspectorDOMStorageResource* cast(const EventListener* listener)
        {
            return listener->type() == InspectorDOMStorageResourceType ? static_cast<const InspectorDOMStorageResource*>(listener) : 0;
        }

        void bind(InspectorFrontend* frontend);
        void unbind();
        void startReportingChangesToFrontend();

        virtual void handleEvent(ScriptExecutionContext*, Event*);
        virtual bool operator==(const EventListener& listener);

        bool isSameHostAndType(Frame*, bool isLocalStorage) const;
        long id() const { return m_id; }
        Storage* domStorage() const { return m_domStorage.get(); }

    private:

        InspectorDOMStorageResource(Storage*, bool isLocalStorage, Frame*);

        RefPtr<Storage> m_domStorage;
        bool m_isLocalStorage;
        RefPtr<Frame> m_frame;
        InspectorFrontend* m_frontend;
        int m_id;
        bool m_reportingChangesToFrontend;

        static int s_nextUnusedId;
    };

} // namespace WebCore

#endif

#endif // InspectorDOMStorageResource_h
