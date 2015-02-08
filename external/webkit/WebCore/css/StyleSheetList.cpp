

#include "config.h"
#include "StyleSheetList.h"

#include "CSSStyleSheet.h"
#include "Document.h"
#include "HTMLNames.h"
#include "HTMLStyleElement.h"
#include "PlatformString.h"

namespace WebCore {

using namespace HTMLNames;

StyleSheetList::StyleSheetList(Document* doc)
    : m_doc(doc)
{
}

StyleSheetList::~StyleSheetList()
{
}

void StyleSheetList::documentDestroyed()
{
    m_doc = 0;
}

unsigned StyleSheetList::length() const
{
    return m_sheets.size();
}

StyleSheet* StyleSheetList::item(unsigned index)
{
    return index < length() ? m_sheets[index].get() : 0;
}

HTMLStyleElement* StyleSheetList::getNamedItem(const String& name) const
{
    if (!m_doc)
        return 0;

    // IE also supports retrieving a stylesheet by name, using the name/id of the <style> tag
    // (this is consistent with all the other collections)
    // ### Bad implementation because returns a single element (are IDs always unique?)
    // and doesn't look for name attribute.
    // But unicity of stylesheet ids is good practice anyway ;)
    
    Element* element = m_doc->getElementById(name);
    if (element && element->hasTagName(styleTag))
        return static_cast<HTMLStyleElement*>(element);
    return 0;
}

} // namespace WebCore
