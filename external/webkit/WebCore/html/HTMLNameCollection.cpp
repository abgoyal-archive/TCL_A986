

#include "config.h"
#include "HTMLNameCollection.h"

#include "Element.h"
#include "HTMLDocument.h"
#include "HTMLNames.h"
#include "HTMLObjectElement.h"

namespace WebCore {

using namespace HTMLNames;

HTMLNameCollection::HTMLNameCollection(PassRefPtr<Document> document, CollectionType type, const String& name)
    : HTMLCollection(document.get(), type, document->nameCollectionInfo(type, name))
    , m_name(name)
{
}

Element* HTMLNameCollection::itemAfter(Element* previous) const
{
    ASSERT(previous != base());

    Node* current;
    if (!previous)
        current = base()->firstChild();
    else
        current = previous->traverseNextNode(base());

    for (; current; current = current->traverseNextNode(base())) {
        if (!current->isElementNode())
            continue;
        Element* e = static_cast<Element*>(current);
        switch (type()) {
            case WindowNamedItems:
                // find only images, forms, applets, embeds and objects by name, 
                // but anything by id
                if (e->hasTagName(imgTag) ||
                    e->hasTagName(formTag) ||
                    e->hasTagName(appletTag) ||
                    e->hasTagName(embedTag) ||
                    e->hasTagName(objectTag))
                    if (e->getAttribute(nameAttr) == m_name)
                        return e;
                if (e->getAttribute(e->idAttributeName()) == m_name)
                    return e;
                break;
            case DocumentNamedItems:
                // find images, forms, applets, embeds, objects and iframes by name, 
                // applets and object by id, and images by id but only if they have
                // a name attribute (this very strange rule matches IE)
                if (e->hasTagName(formTag) || e->hasTagName(embedTag) || e->hasTagName(iframeTag)) {
                    if (e->getAttribute(nameAttr) == m_name)
                        return e;
                } else if (e->hasTagName(appletTag)) {
                    if (e->getAttribute(nameAttr) == m_name || e->getAttribute(e->idAttributeName()) == m_name)
                        return e;
                } else if (e->hasTagName(objectTag)) {
                    if ((e->getAttribute(nameAttr) == m_name || e->getAttribute(e->idAttributeName()) == m_name)
                            && static_cast<HTMLObjectElement*>(e)->isDocNamedItem())
                        return e;
                } else if (e->hasTagName(imgTag)) {
                    if (e->getAttribute(nameAttr) == m_name || (e->getAttribute(e->idAttributeName()) == m_name && e->hasAttribute(nameAttr)))
                        return e;
                }
                break;
        default:
            ASSERT_NOT_REACHED();
        }
    }

    return 0;
}

}
