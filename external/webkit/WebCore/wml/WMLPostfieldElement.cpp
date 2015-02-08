

#include "config.h"

#if ENABLE(WML)
#include "WMLPostfieldElement.h"

#include "CString.h"
#include "TextEncoding.h"
#include "HTMLNames.h"
#include "WMLDocument.h"
#include "WMLGoElement.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLPostfieldElement::WMLPostfieldElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

void WMLPostfieldElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    Node* parent = parentNode();
    if (parent && parent->hasTagName(goTag))
        static_cast<WMLGoElement*>(parent)->registerPostfieldElement(this);
}

void WMLPostfieldElement::removedFromDocument()
{
    Node* parent = parentNode();
    if (parent && parent->hasTagName(goTag))
        static_cast<WMLGoElement*>(parent)->deregisterPostfieldElement(this);

    WMLElement::removedFromDocument();
}

String WMLPostfieldElement::name() const
{
    return parseValueSubstitutingVariableReferences(getAttribute(HTMLNames::nameAttr));
}

String WMLPostfieldElement::value() const
{
    return parseValueSubstitutingVariableReferences(getAttribute(HTMLNames::valueAttr));
}

static inline CString encodedString(const TextEncoding& encoding, const String& data)
{
    return encoding.encode(data.characters(), data.length(), EntitiesForUnencodables);
}

void WMLPostfieldElement::encodeData(const TextEncoding& encoding, CString& name, CString& value)
{
    name = encodedString(encoding, this->name());
    value = encodedString(encoding, this->value());
}

}

#endif
