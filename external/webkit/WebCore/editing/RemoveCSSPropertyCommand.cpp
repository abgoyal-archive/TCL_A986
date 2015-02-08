

#include "config.h"
#include "RemoveCSSPropertyCommand.h"

#include "CSSMutableStyleDeclaration.h"
#include <wtf/Assertions.h>

namespace WebCore {

RemoveCSSPropertyCommand::RemoveCSSPropertyCommand(Document* document, PassRefPtr<CSSMutableStyleDeclaration> style, CSSPropertyID property)
    : SimpleEditCommand(document)
    , m_style(style)
    , m_property(property)
    , m_important(false)
{
    ASSERT(m_style);
}

void RemoveCSSPropertyCommand::doApply()
{
    m_oldValue = m_style->getPropertyValue(m_property);
    m_important = m_style->getPropertyPriority(m_property);
    m_style->removeProperty(m_property);
}

void RemoveCSSPropertyCommand::doUnapply()
{
    m_style->setProperty(m_property, m_oldValue, m_important);
}

} // namespace WebCore
