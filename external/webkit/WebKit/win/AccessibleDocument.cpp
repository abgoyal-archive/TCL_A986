

#include "config.h"
#include "AccessibleDocument.h"

#include <WebCore/AXObjectCache.h>
#include <WebCore/Document.h>
#include <WebCore/RenderObject.h>

using namespace WebCore;

// AccessibleDocument
AccessibleDocument::AccessibleDocument(Document* doc)
    : AccessibleBase(doc->axObjectCache()->getOrCreate(doc->renderer()))
{
}

long AccessibleDocument::role() const
{
    return ROLE_SYSTEM_DOCUMENT;
}

Document* AccessibleDocument::document() const
{
    if (!m_object)
        return 0;
    return m_object->document();
}
