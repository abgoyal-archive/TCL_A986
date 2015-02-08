

#include "config.h"
#include "SetNodeAttributeCommand.h"

#include "Element.h"
#include <wtf/Assertions.h>

namespace WebCore {

SetNodeAttributeCommand::SetNodeAttributeCommand(PassRefPtr<Element> element, 
        const QualifiedName& attribute, const AtomicString& value)
    : SimpleEditCommand(element->document())
    , m_element(element)
    , m_attribute(attribute)
    , m_value(value)
{
    ASSERT(m_element);
}

void SetNodeAttributeCommand::doApply()
{
    m_oldValue = m_element->getAttribute(m_attribute);
    m_element->setAttribute(m_attribute, m_value);
}

void SetNodeAttributeCommand::doUnapply()
{
    m_element->setAttribute(m_attribute, m_oldValue);
    AtomicStringImpl* nullString = 0;
    m_oldValue = nullString;
}

} // namespace WebCore
