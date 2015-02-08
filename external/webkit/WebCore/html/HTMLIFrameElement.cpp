

#include "config.h"
#include "HTMLIFrameElement.h"

#include "CSSPropertyNames.h"
#include "Frame.h"
#include "HTMLDocument.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderPartObject.h"

namespace WebCore {

using namespace HTMLNames;

inline HTMLIFrameElement::HTMLIFrameElement(const QualifiedName& tagName, Document* document)
    : HTMLFrameElementBase(tagName, document)
{
    ASSERT(hasTagName(iframeTag));
}

PassRefPtr<HTMLIFrameElement> HTMLIFrameElement::create(const QualifiedName& tagName, Document* document)
{
    return adoptRef(new HTMLIFrameElement(tagName, document));
}

bool HTMLIFrameElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == widthAttr || attrName == heightAttr) {
        result = eUniversal;
        return false;
    }
    
    if (attrName == alignAttr) {
        result = eReplaced; // Share with <img> since the alignment behavior is the same.
        return false;
    }
    
    if (attrName == frameborderAttr) {
        result = eReplaced;
        return false;
    }

    return HTMLFrameElementBase::mapToEntry(attrName, result);
}

static SandboxFlags parseSandboxAttribute(MappedAttribute* attribute)
{
    if (attribute->isNull())
        return SandboxNone;

    // Parse the unordered set of unique space-separated tokens.
    SandboxFlags flags = SandboxAll;
    const UChar* characters = attribute->value().characters();
    unsigned length = attribute->value().length();
    unsigned start = 0;
    while (true) {
        while (start < length && isASCIISpace(characters[start]))
            ++start;
        if (start >= length)
            break;
        unsigned end = start + 1;
        while (end < length && !isASCIISpace(characters[end]))
            ++end;

        // Turn off the corresponding sandbox flag if it's set as "allowed".
        String sandboxToken = String(characters + start, end - start);
        if (equalIgnoringCase(sandboxToken, "allow-same-origin"))
            flags &= ~SandboxOrigin;
        else if (equalIgnoringCase(sandboxToken, "allow-forms"))
            flags &= ~SandboxForms;
        else if (equalIgnoringCase(sandboxToken, "allow-scripts"))
            flags &= ~SandboxScripts;

        start = end + 1;
    }
    
    return flags;
}

void HTMLIFrameElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == widthAttr)
        addCSSLength(attr, CSSPropertyWidth, attr->value());
    else if (attr->name() == heightAttr)
        addCSSLength(attr, CSSPropertyHeight, attr->value());
    else if (attr->name() == alignAttr)
        addHTMLAlignment(attr);
    else if (attr->name() == nameAttr) {
        const AtomicString& newName = attr->value();
        if (inDocument() && document()->isHTMLDocument()) {
            HTMLDocument* document = static_cast<HTMLDocument*>(this->document());
            document->removeExtraNamedItem(m_name);
            document->addExtraNamedItem(newName);
        }
        m_name = newName;
    } else if (attr->name() == frameborderAttr) {
        // Frame border doesn't really match the HTML4 spec definition for iframes.  It simply adds
        // a presentational hint that the border should be off if set to zero.
        if (!attr->isNull() && !attr->value().toInt())
            // Add a rule that nulls out our border width.
            addCSSLength(attr, CSSPropertyBorderWidth, "0");
    } else if (attr->name() == sandboxAttr)
        setSandboxFlags(parseSandboxAttribute(attr));
    else
        HTMLFrameElementBase::parseMappedAttribute(attr);
}

bool HTMLIFrameElement::rendererIsNeeded(RenderStyle* style)
{
    return isURLAllowed() && style->display() != NONE;
}

RenderObject* HTMLIFrameElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderPartObject(this);
}

void HTMLIFrameElement::insertedIntoDocument()
{
    if (document()->isHTMLDocument())
        static_cast<HTMLDocument*>(document())->addExtraNamedItem(m_name);

    HTMLFrameElementBase::insertedIntoDocument();
}

void HTMLIFrameElement::removedFromDocument()
{
    if (document()->isHTMLDocument())
        static_cast<HTMLDocument*>(document())->removeExtraNamedItem(m_name);

    HTMLFrameElementBase::removedFromDocument();
}

bool HTMLIFrameElement::isURLAttribute(Attribute* attr) const
{
    return attr->name() == srcAttr;
}

}
