

#include "config.h"
#include "HTMLMapElement.h"

#include "Document.h"
#include "HTMLAreaElement.h"
#include "HTMLCollection.h"
#include "HTMLImageElement.h"
#include "HTMLNames.h"
#include "HitTestResult.h"
#include "IntSize.h"
#include "MappedAttribute.h"
#include "RenderObject.h"

using namespace std;

namespace WebCore {

using namespace HTMLNames;

HTMLMapElement::HTMLMapElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(mapTag));
}

HTMLMapElement::~HTMLMapElement()
{
    document()->removeImageMap(this);
}

bool HTMLMapElement::checkDTD(const Node* newChild)
{
    return inEitherTagList(newChild) || newChild->hasTagName(areaTag) // HTML 4 DTD
        || newChild->hasTagName(scriptTag); // extensions
}

bool HTMLMapElement::mapMouseEvent(int x, int y, const IntSize& size, HitTestResult& result)
{
    HTMLAreaElement* defaultArea = 0;
    Node *node = this;
    while ((node = node->traverseNextNode(this))) {
        if (node->hasTagName(areaTag)) {
            HTMLAreaElement* areaElt = static_cast<HTMLAreaElement*>(node);
            if (areaElt->isDefault()) {
                if (!defaultArea)
                    defaultArea = areaElt;
            } else if (areaElt->mapMouseEvent(x, y, size, result))
                return true;
        }
    }
    
    if (defaultArea) {
        result.setInnerNode(defaultArea);
        result.setURLElement(defaultArea);
    }
    return defaultArea;
}

HTMLImageElement* HTMLMapElement::imageElement() const
{
    RefPtr<HTMLCollection> coll = renderer()->document()->images();
    for (Node* curr = coll->firstItem(); curr; curr = coll->nextItem()) {
        if (!curr->hasTagName(imgTag))
            continue;
        
        // The HTMLImageElement's useMap() value includes the '#' symbol at the beginning,
        // which has to be stripped off.
        HTMLImageElement* imageElement = static_cast<HTMLImageElement*>(curr);
        String useMapName = imageElement->getAttribute(usemapAttr).string().substring(1);
        if (equalIgnoringCase(useMapName, m_name))
            return imageElement;
    }
    
    return 0;    
}
    
void HTMLMapElement::parseMappedAttribute(MappedAttribute* attr)
{
    const QualifiedName& attrName = attr->name();
    if (attrName == idAttributeName() || attrName == nameAttr) {
        Document* doc = document();
        if (attrName == idAttributeName()) {
            // Call base class so that hasID bit gets set.
            HTMLElement::parseMappedAttribute(attr);
            if (doc->isHTMLDocument())
                return;
        }
        doc->removeImageMap(this);
        String mapName = attr->value();
        if (mapName[0] == '#')
            mapName = mapName.substring(1);
        m_name = doc->isHTMLDocument() ? mapName.lower() : mapName;
        doc->addImageMap(this);
    } else
        HTMLElement::parseMappedAttribute(attr);
}

PassRefPtr<HTMLCollection> HTMLMapElement::areas()
{
    return HTMLCollection::create(this, MapAreas);
}

String HTMLMapElement::name() const
{
    return getAttribute(nameAttr);
}

void HTMLMapElement::setName(const String& value)
{
    setAttribute(nameAttr, value);
}

}
