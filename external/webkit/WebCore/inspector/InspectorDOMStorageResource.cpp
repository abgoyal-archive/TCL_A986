

#include "config.h"

#if ENABLE(DOM_STORAGE) && ENABLE(INSPECTOR)

#include "InspectorDOMStorageResource.h"

#include "Document.h"
#include "DOMWindow.h"
#include "EventNames.h"
#include "Frame.h"
#include "InspectorFrontend.h"
#include "ScriptObject.h"
#include "Storage.h"
#include "StorageEvent.h"

using namespace JSC;

namespace WebCore {

int InspectorDOMStorageResource::s_nextUnusedId = 1;

InspectorDOMStorageResource::InspectorDOMStorageResource(Storage* domStorage, bool isLocalStorage, Frame* frame)
    :  EventListener(InspectorDOMStorageResourceType)
    , m_domStorage(domStorage)
    , m_isLocalStorage(isLocalStorage)
    , m_frame(frame)
    , m_frontend(0)
    , m_id(s_nextUnusedId++)
    , m_reportingChangesToFrontend(false)
{
}

bool InspectorDOMStorageResource::isSameHostAndType(Frame* frame, bool isLocalStorage) const
{
    return equalIgnoringCase(m_frame->document()->securityOrigin()->host(), frame->document()->securityOrigin()->host()) && m_isLocalStorage == isLocalStorage;
}

void InspectorDOMStorageResource::bind(InspectorFrontend* frontend)
{
    ASSERT(!m_frontend);
    m_frontend = frontend;

    ScriptObject jsonObject = frontend->newScriptObject();
    jsonObject.set("host", m_frame->document()->securityOrigin()->host());
    jsonObject.set("isLocalStorage", m_isLocalStorage);
    jsonObject.set("id", m_id);
    frontend->addDOMStorage(jsonObject);
}

void InspectorDOMStorageResource::unbind()
{
    if (!m_frontend)
        return;  // Already unbound.

    if (m_reportingChangesToFrontend) {
        m_frame->domWindow()->removeEventListener(eventNames().storageEvent, this, true);
        m_reportingChangesToFrontend = false;
    }
    m_frontend = 0;
}

void InspectorDOMStorageResource::startReportingChangesToFrontend()
{
    ASSERT(m_frontend);
    if (!m_reportingChangesToFrontend) {
        m_frame->domWindow()->addEventListener(eventNames().storageEvent, this, true);
        m_reportingChangesToFrontend = true;
    }
}

void InspectorDOMStorageResource::handleEvent(ScriptExecutionContext*, Event* event)
{
    ASSERT(m_frontend);
    ASSERT(eventNames().storageEvent == event->type());
    StorageEvent* storageEvent = static_cast<StorageEvent*>(event);
    Storage* storage = storageEvent->storageArea();
    bool isLocalStorage = storage->frame()->domWindow()->localStorage() == storage;
    if (isSameHostAndType(storage->frame(), isLocalStorage))
        m_frontend->updateDOMStorage(m_id);
}

bool InspectorDOMStorageResource::operator==(const EventListener& listener)
{
    return (this == InspectorDOMStorageResource::cast(&listener));
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE) && ENABLE(INSPECTOR)

