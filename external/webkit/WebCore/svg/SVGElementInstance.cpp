

#include "config.h"

#if ENABLE(SVG)
#include "SVGElementInstance.h"

#include "ContainerNodeAlgorithms.h"
#include "Event.h"
#include "EventException.h"
#include "EventListener.h"
#include "EventNames.h"
#include "FrameView.h"
#include "SVGElementInstanceList.h"
#include "SVGUseElement.h"

#include <wtf/RefCountedLeakCounter.h>

namespace WebCore {

#ifndef NDEBUG
static WTF::RefCountedLeakCounter instanceCounter("WebCoreSVGElementInstance");
#endif

SVGElementInstance::SVGElementInstance(SVGUseElement* useElement, PassRefPtr<SVGElement> originalElement)
    : m_useElement(useElement)
    , m_element(originalElement)
    , m_previousSibling(0)
    , m_nextSibling(0)
    , m_firstChild(0)
    , m_lastChild(0)
{
    ASSERT(m_useElement);
    ASSERT(m_element);

    // Register as instance for passed element.
    m_element->mapInstanceToElement(this);

#ifndef NDEBUG
    instanceCounter.increment();
#endif
}

SVGElementInstance::~SVGElementInstance()
{
#ifndef NDEBUG
    instanceCounter.decrement();
#endif

    // Deregister as instance for passed element.
    m_element->removeInstanceMapping(this);

    removeAllChildrenInContainer<SVGElementInstance, SVGElementInstance>(this);
}

PassRefPtr<SVGElementInstanceList> SVGElementInstance::childNodes()
{
    return SVGElementInstanceList::create(this);
}

void SVGElementInstance::setShadowTreeElement(SVGElement* element)
{
    ASSERT(element);
    m_shadowTreeElement = element;
}

void SVGElementInstance::appendChild(PassRefPtr<SVGElementInstance> child)
{
    appendChildToContainer<SVGElementInstance, SVGElementInstance>(child.get(), this);
}

void SVGElementInstance::invalidateAllInstancesOfElement(SVGElement* element)
{
    if (!element)
        return;

    if (element->isStyled() && static_cast<SVGStyledElement*>(element)->instanceUpdatesBlocked())
        return;

    const HashSet<SVGElementInstance*>& set = element->instancesForElement();
    if (set.isEmpty())
        return;

    // Mark all use elements referencing 'element' for rebuilding
    const HashSet<SVGElementInstance*>::const_iterator end = set.end();
    for (HashSet<SVGElementInstance*>::const_iterator it = set.begin(); it != end; ++it) {
        ASSERT((*it)->correspondingElement() == element);
        (*it)->correspondingUseElement()->invalidateShadowTree();
    }
}

ScriptExecutionContext* SVGElementInstance::scriptExecutionContext() const
{
    return m_element->document();
}

bool SVGElementInstance::addEventListener(const AtomicString& eventType, PassRefPtr<EventListener> listener, bool useCapture)
{
    return m_element->addEventListener(eventType, listener, useCapture);
}

bool SVGElementInstance::removeEventListener(const AtomicString& eventType, EventListener* listener, bool useCapture)
{
    return m_element->removeEventListener(eventType, listener, useCapture);
}

void SVGElementInstance::removeAllEventListeners()
{
    m_element->removeAllEventListeners();
}

bool SVGElementInstance::dispatchEvent(PassRefPtr<Event> prpEvent)
{
    RefPtr<EventTarget> protect = this;
    RefPtr<Event> event = prpEvent;

    event->setTarget(this);

    SVGElement* element = shadowTreeElement();
    if (!element)
        return false;

    RefPtr<FrameView> view = element->document()->view();
    return element->dispatchGenericEvent(event.release());
}

EventTargetData* SVGElementInstance::eventTargetData()
{
    return m_element->eventTargetData();
}

EventTargetData* SVGElementInstance::ensureEventTargetData()
{
    // Avoid crashing - return a default dummy value
    DEFINE_STATIC_LOCAL(EventTargetData, dummyEventTargetData, ());
    dummyEventTargetData.eventListenerMap.clear();
    return &dummyEventTargetData;
}

}

#endif
